"""
Utility functions for motion planning algorithms.
This module provides helper functions for:
1. Collision detection with various obstacle types (circles, rectangles, boundaries)
2. Geometric calculations (intersections, distances, rays)
3. System dynamics integration (single and double integrators)

Authors: 
- Tirth Sadaria
- Kunj Golwala
- Rahul Kumar
Date: 16th May 2025
"""

import math
import numpy as np
import os
import sys

import env
from rrt import Node


class Utils:
    """
    Utility class providing helper functions for motion planning.
    Handles collision checking, geometric calculations, and system dynamics.
    """
    def __init__(self):
        """
        Initialize utility class with environment and safety parameters.
        """
        self.env = env.Env()

        self.delta = 0.5  # Base safety margin
        self.obs_circle = self.env.obs_circle  # Circular obstacles
        self.obs_rectangle = self.env.obs_rectangle  # Rectangular obstacles
        self.obs_boundary = self.env.obs_boundary  # Boundary obstacles
        self.clearance = 4.0  # Additional safety margin for obstacle avoidance

        self.obs_points = self.env.obs_points  # Point obstacles

    def update_obs(self, obs_cir, obs_bound, obs_rec):
        """
        Update obstacle lists with new values.
        
        Args:
            obs_cir: New circular obstacles
            obs_bound: New boundary obstacles
            obs_rec: New rectangular obstacles
        """
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec
        self.obs_points = obs_points
        self.delta = 0.5

    def get_obs_vertex(self):
        """
        Get vertices of rectangular obstacles with safety margins.
        
        Returns:
            List of vertex lists for each rectangular obstacle
        """
        delta = self.delta + self.clearance
        obs_list = []

        for ox, oy, w, h in self.obs_rectangle:
            # Create vertices with safety margin
            vertex_list = [
                [ox - delta, oy - delta],
                [ox + w + delta, oy - delta],
                [ox + w + delta, oy + h + delta],
                [ox - delta, oy + h + delta],
            ]
            obs_list.append(vertex_list)

        return obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):
        """
        Check if a line segment intersects with a rectangle edge.
        Uses vector cross product for intersection detection.
        
        Args:
            start: Start node of line segment
            end: End node of line segment
            o: Origin point
            d: Direction vector
            a: First vertex of rectangle edge
            b: Second vertex of rectangle edge
            
        Returns:
            bool: True if intersection exists
        """
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_intersect_circle(self, o, d, a, r):
        """
        Check if a line segment intersects with a circle.
        Uses parametric line-circle intersection.
        
        Args:
            o: Origin point of line segment
            d: Direction vector
            a: Center of circle
            r: Radius of circle
            
        Returns:
            bool: True if intersection exists
        """
        d2 = np.dot(d, d)
        delta = self.delta

        if d2 == 0:
            return False

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        if 0 <= t <= 1:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            if self.get_dist(shot, Node(a)) <= r + delta:
                return True
        return False

    def is_collision(self, start, end):
        """
        Check if path between start and end nodes collides with any obstacles.
        Checks collisions with all obstacle types: circles, rectangles, boundaries, and points.
        
        Args:
            start: Start node
            end: End node
            
        Returns:
            bool: True if collision detected
        """
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obs_vertex()

        # Check rectangle collisions
        for v1, v2, v3, v4 in obs_vertex:
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True

        # Check circle collisions
        for x, y, r in self.obs_circle:
            if self.is_intersect_circle(o, d, [x, y], r):
                return True
        
        # Check point obstacle collisions
        if len(self.obs_points) > 1:
            for i in range(len(self.obs_points) - 1):
                if self.is_intersect_rec(
                    start,
                    end,
                    o,
                    d,
                    [self.obs_points[i][0], self.obs_points[i][1]],
                    [self.obs_points[i + 1][0], self.obs_points[i + 1][1]],
                ):
                    return True

        return False

    def is_inside_obs(self, node):
        """
        Check if a node is inside any obstacle (including safety margins).
        
        Args:
            node: Node to check
            
        Returns:
            bool: True if node is inside any obstacle
        """
        delta = self.delta + self.clearance

        # Check circle obstacles
        for x, y, r in self.obs_circle:
            if math.hypot(node.x - x, node.y - y) <= r + delta:
                return True

        # Check rectangle obstacles
        for x, y, w, h in self.obs_rectangle:
            if (
                0 <= node.x - (x - delta) <= w + 2 * delta
                and 0 <= node.y - (y - delta) <= h + 2 * delta
            ):
                return True

        # Check boundary obstacles
        for x, y, w, h in self.obs_boundary:
            if (
                0 <= node.x - (x - delta) <= w + 2 * delta
                and 0 <= node.y - (y - delta) <= h + 2 * delta
            ):
                return True

        # Check point obstacles
        for pt in self.obs_points:
            if (
                0<=node.x - (pt[0] - delta) <= delta
                and 0<=node.y - (pt[1] - delta) <= delta
            ):
                return True

        return False

    @staticmethod
    def get_ray(start, end):
        """
        Get ray parameters (origin and direction) from start to end node.
        
        Args:
            start: Start node
            end: End node
            
        Returns:
            Tuple of (origin, direction) vectors
        """
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        """
        Calculate Euclidean distance between two nodes.
        
        Args:
            start: Start node
            end: End node
            
        Returns:
            float: Euclidean distance
        """
        return math.hypot(end.x - start.x, end.y - start.y)

    @staticmethod
    def integrate_single_integrator(self, x_init, u, dt):
        """
        Integrate single integrator dynamics: ẋ = u.
        State: [x1, x2]^T (position)
        Control: [u1, u2]^T (velocity)
        
        Args:
            x_init: Initial state
            u: Control sequence
            dt: Time step
            
        Returns:
            Array of state trajectory
        """
        x_traj = np.array([x_init])
        x_current = x_traj.copy()
        num_steps = len(u)

        for i in range(num_steps):
            u_current = u[i]
            x_current[0, 0] = x_current[0, 1] + dt * u_current[0]
            x_current[0, 1] = x_current[0, 1] + dt * u_current[1]
            x_traj = np.concatenate((x_traj, x_current), axis=0)
        return x_traj

    @staticmethod
    def integrate_double_integrator(x_init, u, dt):
        """
        Integrate double integrator dynamics: ẍ = u.
        State: [x1, v1, x2, v2]^T (position and velocity)
        Control: [u1, u2]^T (acceleration)
        
        Args:
            x_init: Initial state
            u: Control sequence
            dt: Time step
            
        Returns:
            Array of state trajectory
        """
        x_traj = np.array([x_init], dtype=np.float32)
        x_current = x_traj.copy()
        num_steps = len(u)

        for i in range(num_steps):
            u_current = u[i]
            # Position update with acceleration
            x_current[0, 0] = (
                x_current[0, 0] + x_current[0, 1] * dt + 0.5 * u_current[0] * dt**2
            )
            # Velocity update
            x_current[0, 1] = x_current[0, 1] + dt * u_current[0]
            # Position update with acceleration
            x_current[0, 2] = (
                x_current[0, 2] + x_current[0, 3] * dt + 0.5 * u_current[1] * dt**2
            )
            # Velocity update
            x_current[0, 3] = x_current[0, 3] + dt * u_current[1]
            x_traj = np.concatenate((x_traj, x_current), axis=0)

        return x_traj
