"""
Plotting tools for Sampling-based algorithms.
This module provides visualization capabilities for:
1. Environment with obstacles (boundary, rectangular, circular, and point obstacles)
2. Search trees and visited nodes
3. Planned paths and trajectories
4. Real-time animation of the planning process

Authors: 
- Tirth Sadaria
- Kunj Golwala
- Rahul Kumar
Date: 16th May 2025
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import sys
import env


class Plotting:
    """
    Class for visualizing motion planning algorithms and their results.
    Handles plotting of environment, obstacles, search trees, and planned paths.
    """
    def __init__(self, x_start, x_goal):
        """
        Initialize plotting environment with start and goal positions.
        
        Args:
            x_start: Start position coordinates [x, y]
            x_goal: Goal position coordinates [x, y]
        """
        self.xI, self.xG = x_start, x_goal  # Start and goal positions
        self.env = env.Env()  # Environment instance
        # Load different types of obstacles
        self.obs_bound = self.env.obs_boundary  # Boundary obstacles
        self.obs_circle = self.env.obs_circle   # Circular obstacles
        self.obs_rectangle = self.env.obs_rectangle  # Rectangular obstacles
        self.obs_points = self.env.obs_points   # Point obstacles

    def animation(self, nodelist, path, name, animation=False):
        """
        Create animation of the planning process and final path.
        
        Args:
            nodelist: List of nodes in the search tree
            path: Planned path coordinates
            name: Title for the plot
            animation: Flag to enable/disable animation
        """
        self.plot_grid(name)  # Plot environment and obstacles
        self.plot_visited(nodelist, animation)  # Plot search tree
        self.plot_path(path)  # Plot final path

    def animation_online(self, nodelist, name, animation=False):
        """
        Create real-time animation of the planning process.
        
        Args:
            nodelist: List of nodes in the search tree
            name: Title for the plot
            animation: Flag to enable/disable animation
        """
        self.plot_grid(name)  # Plot environment and obstacles
        self.plot_visited(nodelist, animation)  # Plot search tree
        plt.pause(1.0)
        plt.close()

    def animation_connect(self, V1, V2, path, name):
        """
        Create animation showing connection between two search trees.
        
        Args:
            V1: First set of nodes
            V2: Second set of nodes
            path: Planned path coordinates
            name: Title for the plot
        """
        self.plot_grid(name)  # Plot environment and obstacles
        self.plot_visited_connect(V1, V2)  # Plot connected search trees
        self.plot_path(path)  # Plot final path

    def plot_grid(self, name):
        """
        Plot the environment grid with obstacles and start/goal positions.
        
        Args:
            name: Title for the plot
        """
        fig, ax = plt.subplots()

        # Plot boundary obstacles
        for ox, oy, w, h in self.obs_bound:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h, edgecolor="black", facecolor="black", fill=True
                )
            )

        # Plot rectangular obstacles with clearance
        clearance = 6  # Safety margin around obstacles
        for ox, oy, w, h in self.obs_rectangle:
            # Plot actual obstacle
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h, edgecolor="black", facecolor="gray", fill=True
                )
            )
            # Plot clearance zone
            ax.add_patch(
                patches.Rectangle(
                    (ox - clearance, oy - clearance),
                    w + 2 * clearance,
                    h + 2 * clearance,
                    edgecolor="red",
                    facecolor="gray",
                    fill=True
                )
            )

        # Plot circular obstacles
        for ox, oy, r in self.obs_circle:
            ax.add_patch(
                patches.Circle(
                    (ox, oy), r, edgecolor="black", facecolor="gray", fill=True
                )
            )

        # for pt in self.obs_points:
        #     ox, oy = pt[0], pt[1]   
        #     ax.add_patch(
        #         patches.Circle(
        #             (ox, oy), 0.5, edgecolor="black", facecolor="gray", fill=True
        #         )
        #     )
        if len(self.obs_points) > 1:
            for i in range(len(self.obs_points) - 1):
                x_values = [self.obs_points[i][0], self.obs_points[i+1][0]]
                y_values = [self.obs_points[i][1], self.obs_points[i+1][1]]
                plt.plot(x_values, y_values, '-k', linewidth=2)

        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3)
        plt.plot(self.xG[0], self.xG[1], "rs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    @staticmethod
    def plot_visited(nodelist, animation):
        """
        Plot the search tree (visited nodes and their connections).
        
        Args:
            nodelist: List of nodes in the search tree
            animation: Flag to enable/disable animation
        """
        if animation:
            # Animated plotting of search tree
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect(
                        "key_release_event",
                        lambda event: [exit(0) if event.key == "escape" else None],
                    )
                    if count % 10 == 0:
                        plt.pause(1.0)
        else:
            # Static plotting of search tree
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plot_visited_connect(V1, V2):
        """
        Plot two connected search trees with animation.
        
        Args:
            V1: First set of nodes
            V2: Second set of nodes
        """
        len1, len2 = len(V1), len(V2)

        # Plot nodes and connections from both trees
        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            # Enable escape key to exit
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )

            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)

    @staticmethod
    def plot_path(path):
        """
        Plot the final planned path.
        
        Args:
            path: List of coordinates representing the planned path
        """
        if len(path) != 0:
            # Plot path in red with increased line width
            plt.plot([x[0] for x in path], [x[1] for x in path], "-r", linewidth=2)
            plt.pause(0.01)
            # Save the final plot
            plt.savefig("LQR-CBF_result.PNG")
        plt.show()
