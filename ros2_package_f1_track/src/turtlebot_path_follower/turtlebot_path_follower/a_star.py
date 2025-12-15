#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from math import sqrt, cos, sin, atan2, pi
import heapq
import cv2

# Constants
WHEEL_RADIUS = 3.3  # cm
WHEEL_DISTANCE = 29.0  # cm
ROBOT_RADIUS = 22.0  # mm (to be scaled)
CANVAS_HEIGHT = 300
CANVAS_WIDTH = 540

class NodeObj:
    def __init__(self, coords, cost, parent=None, heuristic=0):
        self.coords = coords
        self.x = coords[0]
        self.y = coords[1]
        self.theta = coords[2]
        self.cost = cost
        self.parent = parent
        self.heuristic = heuristic

    def __lt__(self, other):
        return self.cost + self.heuristic < other.cost + other.heuristic


def deg2rad(deg):
    return deg * pi / 180.0

def round_value(number):
    return np.round(number * 2.0) / 2.0

def is_valid(x, y, canvas):
    return 0 <= x < CANVAS_WIDTH and 0 <= y < CANVAS_HEIGHT and canvas[int(y)][int(x)] == 0

def generate_obstacle_map(clearance):
    grid = np.zeros((CANVAS_HEIGHT, CANVAS_WIDTH), dtype=np.uint8)
    offset = clearance + ROBOT_RADIUS

    def is_obstacle(x, y):
        return (
            (100 <= x <= 103 and 100 <= y <= 299) or
            (210 <= x <= 213 and 100 <= y <= 299) or
            (320 <= x <= 323 and 200 <= y <= 299) or
            (320 <= x <= 323 and 0 <= y <= 199) or
            (430 <= x <= 433 and 100 <= y <= 299)
        )

    for i in range(CANVAS_HEIGHT):
        for j in range(CANVAS_WIDTH):
            if is_obstacle(j, CANVAS_HEIGHT - i):
                grid[i][j] = 1

    kernel = np.ones((2*offset+1, 2*offset+1), np.uint8)
    inflated = cv2.dilate(grid, kernel, iterations=1)
    return inflated

def differential_drive(x, y, theta, UL, UR, dt=0.05, steps=20):
    t = 0
    theta = deg2rad(theta)
    Xn, Yn, Thetan = x, y, theta
    for _ in range(steps):
        t += dt
        ul = (UL * 2 * pi) / 60
        ur = (UR * 2 * pi) / 60
        dx = 0.5 * WHEEL_RADIUS * (ul + ur) * cos(Thetan) * dt
        dy = 0.5 * WHEEL_RADIUS * (ul + ur) * sin(Thetan) * dt
        dtheta = (WHEEL_RADIUS / WHEEL_DISTANCE) * (ur - ul) * dt
        Xn += dx
        Yn += dy
        Thetan += dtheta
    return [Xn, Yn, Thetan * 180 / pi]

def generate_successors(node, canvas, RPM1, RPM2):
    successors = []
    x, y, theta = node.coords
    cost = node.cost
    actions = [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]
    for ul, ur in actions:
        new_coords = differential_drive(x, y, theta, ul, ur)
        xn, yn, thetan = new_coords
        if is_valid(xn, yn, canvas):
            rounded_coords = [round_value(xn), round_value(yn), thetan % 360]
            new_cost = cost + sqrt((xn - x)**2 + (yn - y)**2)
            successors.append([rounded_coords, new_cost])
    return successors

def backtrack(goal_node):
    path = []
    current = goal_node
    while current:
        path.append([int(current.x), int(current.y), int(current.theta)])
        current = current.parent
    return path[::-1]

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner_node')
        self.declare_parameter('start_x', 0)
        self.declare_parameter('start_y', 0)
        self.declare_parameter('start_theta', 0)
        self.declare_parameter('goal_x', 100)
        self.declare_parameter('goal_y', 100)
        self.declare_parameter('RPM1', 10)
        self.declare_parameter('RPM2', 20)
        self.declare_parameter('clearance', 5)

        self.timer = self.create_timer(1.0, self.run_planner)

    def run_planner(self):
        self.timer.cancel()

        start = [
            self.get_parameter('start_x').get_parameter_value().integer_value,
            self.get_parameter('start_y').get_parameter_value().integer_value,
            self.get_parameter('start_theta').get_parameter_value().integer_value
        ]
        goal = [
            self.get_parameter('goal_x').get_parameter_value().integer_value,
            self.get_parameter('goal_y').get_parameter_value().integer_value,
            0
        ]
        RPM1 = self.get_parameter('RPM1').get_parameter_value().integer_value
        RPM2 = self.get_parameter('RPM2').get_parameter_value().integer_value
        clearance = self.get_parameter('clearance').get_parameter_value().integer_value

        canvas = generate_obstacle_map(clearance)
        start_node = NodeObj(start, 0)
        goal_node = NodeObj(goal, 0)

        open_list = []
        heapq.heappush(open_list, (0, start_node))
        visited = set()

        while open_list:
            _, current_node = heapq.heappop(open_list)
            key = (round_value(current_node.x), round_value(current_node.y))
            if key in visited:
                continue
            visited.add(key)

            if sqrt((current_node.x - goal_node.x)**2 + (current_node.y - goal_node.y)**2) < 2.0:
                self.get_logger().info('Goal reached!')
                path = backtrack(current_node)
                self.get_logger().info(f"Final path with {len(path)} waypoints:")
                for p in path:
                    self.get_logger().info(str(p))
                return

            for child, new_cost in generate_successors(current_node, canvas, RPM1, RPM2):
                child_node = NodeObj(child, new_cost, current_node)
                heuristic = sqrt((child[0] - goal_node.x)**2 + (child[1] - goal_node.y)**2)
                child_node.heuristic = heuristic
                heapq.heappush(open_list, (child_node.cost + heuristic, child_node))

        self.get_logger().warn('Path not found.')

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()