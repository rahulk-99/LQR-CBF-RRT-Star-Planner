#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import math
import os
from ament_index_python.packages import get_package_share_directory

class TurtleBotAbsolutePathFollower(Node):
    def __init__(self):
        super().__init__('turtlebot_absolute_path_follower')
        # Load parameter
        pkg_dir = get_package_share_directory('turtlebot_path_follower')
        default_file = os.path.join(pkg_dir, 'data', 'final_path.txt')
        self.declare_parameter('path_file', default_file)
        self.path_file = self.get_parameter('path_file').get_parameter_value().string_value

        # Read absolute waypoints (cm) and convert to meters
        abs_pts = self.load_waypoints_cm(self.path_file)
        self.waypoints = [(x_cm / 10.0, y_cm / 10.0) for x_cm, y_cm in abs_pts]

        # We assume robot is spawned at waypoint[0], so start at waypoint[1]
        self.index = 1
        self.segment_target = None

        self.curr_pose = None
        self.curr_theta = 0.0

        # ROS interfaces
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} absolute waypoints; starting at waypoint 2.'
        )

    def load_waypoints_cm(self, file_path):
        pts = []
        with open(file_path, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) >= 2:
                    pts.append((float(parts[0]), float(parts[1])))
        return pts

    def odom_callback(self, msg):
        # Update current pose and yaw
        self.curr_pose = msg.pose.pose.position
        rot = msg.pose.pose.orientation
        _, _, self.curr_theta = quat2euler(
            [rot.w, rot.x, rot.y, rot.z], axes='sxyz'
        )

        # On first odom, set first target
        if self.segment_target is None and self.index < len(self.waypoints):
            self.segment_target = self.waypoints[self.index]
            self.get_logger().info(
                f'Starting to waypoint {self.index+1}: {self.segment_target}'
            )

    def control_loop(self):
        if self.curr_pose is None or self.segment_target is None:
            return

        goal_x, goal_y = self.segment_target
        dx = goal_x - self.curr_pose.x
        dy = goal_y - self.curr_pose.y
        dist = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.curr_theta)
        
        self.get_logger().info(
            f'Current pose: ({self.curr_pose.x:.2f}, {self.curr_pose.y:.2f}), '
            f'Target: {self.segment_target}, '
            f'Distance: {dist:.2f}, Angle diff: {angle_diff:.2f}'
        )
        cmd = Twist()
        if abs(angle_diff) > 0.1:
            cmd.angular.z = 2.0 * angle_diff
        else:
            cmd.linear.x = 0.8

        # Check arrival
        if dist < 0.3:
            self.get_logger().info(
                f'Reached waypoint {self.index+1}/{len(self.waypoints)}'
            )
            self.index += 1
            if self.index < len(self.waypoints):
                self.segment_target = self.waypoints[self.index]
                self.get_logger().info(
                    f'Moving to waypoint {self.index+1}: {self.segment_target}'
                )
            else:
                self.get_logger().info('All waypoints reached.')
                self.cmd_vel_pub.publish(Twist())
                rclpy.shutdown()
                return

        self.cmd_vel_pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotAbsolutePathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()