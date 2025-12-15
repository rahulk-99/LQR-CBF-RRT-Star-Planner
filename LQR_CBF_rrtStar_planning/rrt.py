"""
Rapidly-exploring Random Tree (RRT) implementation for motion planning.
This module implements a basic RRT algorithm for path planning in 2D environments with obstacles.
The algorithm incrementally builds a tree by randomly sampling the configuration space
and connecting new nodes to the nearest existing node while avoiding obstacles.

Authors: 
- Tirth Sadaria
- Kunj Golwala
- Rahul Kumar
Date: 16th May 2025
"""

import os
import sys
import math
import numpy as np

import env, plotting, utils


class Node:
    """
    Node class representing a point in the configuration space.
    Each node stores its position and parent node for path reconstruction.
    """
    def __init__(self, n):
        """
        Initialize a node with given coordinates.
        
        Args:
            n: Tuple of (x, y) coordinates
        """
        self.x = n[0]  # x-coordinate
        self.y = n[1]  # y-coordinate
        self.parent = None  # Parent node for path reconstruction


class Rrt:
    """
    Rapidly-exploring Random Tree (RRT) implementation.
    Builds a tree incrementally by sampling random points and connecting them
    to the nearest existing node while avoiding obstacles.
    """
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        """
        Initialize RRT planner with start and goal positions.
        
        Args:
            s_start: Start position coordinates (x, y)
            s_goal: Goal position coordinates (x, y)
            step_len: Maximum step length for new node generation
            goal_sample_rate: Probability of sampling the goal position
            iter_max: Maximum number of iterations for tree construction
        """
        self.s_start = Node(s_start)  # Start node
        self.s_goal = Node(s_goal)    # Goal node
        self.step_len = step_len      # Maximum step length
        self.goal_sample_rate = goal_sample_rate  # Goal sampling probability
        self.iter_max = iter_max      # Maximum iterations
        self.vertex = [self.s_start]  # List of all nodes in the tree

        # Initialize environment and utilities
        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()

        # Environment parameters
        self.x_range = self.env.x_range  # x-axis boundaries
        self.y_range = self.env.y_range  # y-axis boundaries
        self.obs_circle = self.env.obs_circle  # Circular obstacles
        self.obs_rectangle = self.env.obs_rectangle  # Rectangular obstacles
        self.obs_boundary = self.env.obs_boundary  # Boundary obstacles

    def planning(self):
        """
        Main RRT planning algorithm.
        Incrementally builds the tree until goal is reached or max iterations.
        
        Returns:
            path: List of (x, y) coordinates forming the path to goal, or None if no path found
        """
        for i in range(self.iter_max):
            # Generate random node
            node_rand = self.generate_random_node(self.goal_sample_rate)
            # Find nearest node in tree
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            # Generate new node towards random node
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                # Check if goal is reachable
                if dist <= self.step_len and not self.utils.is_collision(
                    node_new, self.s_goal
                ):
                    self.new_state(node_new, self.s_goal)
                    return self.extract_path(node_new)
        return None

    def generate_random_node(self, goal_sample_rate):
        """
        Generate a random node in the configuration space.
        With probability goal_sample_rate, returns the goal node.
        
        Args:
            goal_sample_rate: Probability of sampling the goal position
            
        Returns:
            Node: Random node or goal node
        """
        delta = self.utils.delta  # Safety margin from boundaries

        if np.random.random() > goal_sample_rate:
            # Generate random node within workspace bounds
            return Node(
                (
                    np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                    np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta),
                )
            )

        return self.s_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        """
        Find the nearest node in the tree to the given node.
        
        Args:
            node_list: List of nodes in the tree
            n: Target node
            
        Returns:
            Node: Nearest node in the tree
        """
        return node_list[
            int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y) for nd in node_list]))
        ]

    def new_state(self, node_start, node_end):
        """
        Generate a new node towards node_end from node_start.
        
        Args:
            node_start: Starting node
            node_end: Target node
            
        Returns:
            Node: New node towards node_end
        """
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        # Limit step length
        dist = min(self.step_len, dist)
        node_new = Node(
            (
                node_start.x + dist * math.cos(theta),
                node_start.y + dist * math.sin(theta),
            )
        )
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        """
        Extract path from goal to start by following parent pointers.
        
        Args:
            node_end: End node of the path
            
        Returns:
            List of (x, y) coordinates forming the path
        """
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        """
        Calculate distance and angle between two nodes.
        
        Args:
            node_start: Starting node
            node_end: End node
            
        Returns:
            Tuple of (distance, angle) between nodes
        """
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def main():
    """
    Main function demonstrating RRT path planning.
    Creates an RRT planner and visualizes the resulting path.
    """
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node

    # Initialize RRT planner
    rrt = Rrt(x_start, x_goal, 0.5, 0.05, 10000)
    path = rrt.planning()

    if path:
        print("Path Found!")
        rrt.plotting.animation(rrt.vertex, path, "RRT", True)
    else:
        print("No Path Found!")


if __name__ == "__main__":
    main()
