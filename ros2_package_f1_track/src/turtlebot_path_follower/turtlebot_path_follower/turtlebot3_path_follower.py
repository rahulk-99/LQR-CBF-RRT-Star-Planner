#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import math
import os
from ament_index_python.packages import get_package_share_directory

class TurtleBotPathFollower(Node):
    def __init__(self):
        super().__init__('turtlebot_path_follower')
        # Parameters
        # pkg_dir = get_package_share_directory('turtlebot_path_follower')
        pkg_dir = get_package_share_directory('turtlebot_path_follower')
        self.path_file = os.path.join(pkg_dir, 'data', 'final_path.txt')
        # self.path_file = '/home/rahulk99/Downloads/final_path.txt'
        self.declare_parameter('path_file', self.path_file)
        self.path_file = self.get_parameter('path_file').get_parameter_value().string_value
        # Waypoint list
        self.waypoints = self.load_waypoints(self.path_file)
        self.curr_pose = None
        self.curr_theta = 0.0
        self.index = 0
        self.canvas_height = 3.0
        # Subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)

    def load_waypoints(self, file_path):
        waypoints = []
        with open(file_path, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) >= 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    waypoints.append((x, y))
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints.')
        return waypoints

    def odom_callback(self, msg):
        self.curr_pose = msg.pose.pose.position
        rot_q = msg.pose.pose.orientation
        _, _, self.curr_theta = quat2euler([
            rot_q.w, rot_q.x, rot_q.y, rot_q.z  # Note the order for transforms3d: [w, x, y, z]
            ], axes='sxyz')
        self.get_logger().info(f"Current Pose: x={self.curr_pose.x:.2f}, y={self.curr_pose.y:.2f}, theta={math.degrees(self.curr_theta):.1f}")

    def control_loop(self):
        if self.curr_pose is None or self.index >= len(self.waypoints):
            return
        
        goal_x, goal_y = self.waypoints[self.index]
        goal_x = goal_x / 100
        goal_y = goal_y / 100
        goal_y = self.canvas_height - goal_y
        goal_y = goal_y - (self.canvas_height / 2)
        goal_x = goal_x
        dx = goal_x - self.curr_pose.x
        dy = goal_y - self.curr_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.curr_theta)
        cmd = Twist()
        # Rotation threshold
        if abs(angle_diff) > 0.1:
            cmd.angular.z = 2.0 * angle_diff
        else:
            cmd.linear.x = 0.4
            # cmd.angular.z = 0.2 * angle_diff
        
        # Check if waypoint reached
        if distance < 0.1:
            self.get_logger().info(f'Reached waypoint {self.index + 1}/{len(self.waypoints)}')
            self.index += 1
            if self.index == len(self.waypoints):
                self.get_logger().info("All waypoints reached.")
                self.cmd_vel_pub.publish(Twist())  # Stop
                rclpy.shutdown()
                return

        self.cmd_vel_pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotPathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

