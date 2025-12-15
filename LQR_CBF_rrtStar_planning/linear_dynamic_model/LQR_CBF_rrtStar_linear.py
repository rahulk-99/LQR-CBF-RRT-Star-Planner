import os
import sys
import math
import numpy as np
import time
import timeit
import matplotlib.pyplot as plt
from sklearn.neighbors import KernelDensity

import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent))
import env, plotting, utils, Queue
from LQR_planning import LQRPlanner

import copy
import time

"""
LQR_CBF_RRT* (Linear Version)

This module implements a 2D LQR-CBF-RRT* path planning algorithm with adaptive sampling and control barrier function (CBF) constraints. The planner uses LQR-based steering and rewiring, and can adaptively sample the state space using kernel density estimation (KDE) based on elite trajectories. The code is designed for research and educational purposes in motion planning and optimal control.

Authors:
    Tirth Sadaria
    Kunj Golwala
    Rahul Kumar
    
Date: 16/05/2025
"""


class Node:
    """
    Represents a node in the RRT* tree.
    Each node contains its (x, y) position, parent node, cost-to-come, trajectory from parent, and children indices.
    """
    def __init__(self, n):
        self.x = n[0]  # X-coordinate of the node
        self.y = n[1]  # Y-coordinate of the node
        self.parent = None  # Parent node in the tree
        self.cost = 0  # Cost-to-come from the start node
        self.StateTraj = None  # State trajectory from parent to this node
        self.u_parent_to_current = None  # Control input from parent to this node
        self.childrenNodeInds = set([])  # Indices of child nodes (for rewiring)


class LQRrrtStar:
    """
    LQRrrtStar implements the LQR-CBF-RRT* path planning algorithm in 2D.
    This class supports adaptive sampling using kernel density estimation (KDE),
    LQR-based steering, and rewiring with optional control barrier function (CBF) constraints.
    """
    def __init__(
        self,
        x_start,
        x_goal,
        step_len,
        goal_sample_rate,
        search_radius,
        iter_max,
        AdSamplingFlag=False,
        solve_QP=False,
    ):
        """
        Initialize the LQRrrtStar planner with environment, planning, and sampling parameters.

        Args:
            x_start (tuple): Start position (x, y).
            x_goal (tuple): Goal position (x, y).
            step_len (float): Maximum step length for tree extension.
            goal_sample_rate (float): Probability of sampling the goal during random sampling.
            search_radius (float): Radius for neighbor search during rewiring.
            iter_max (int): Maximum number of iterations for planning.
            AdSamplingFlag (bool): Enable adaptive sampling (default: False).
            solve_QP (bool): Enable QP-based CBF constraints (default: False).
        """
        self.s_start = Node(x_start)
        self.s_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_len = 8
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.vertex = [self.s_start]
        self.path = []
        self.u_path = []  # Open-loop control input

        self.env = env.Env()
        self.plotting = plotting.Plotting(x_start, x_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_points = self.env.obs_points
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.lqr_planner = LQRPlanner()
        self.solve_QP = solve_QP

        # The adaptive sampling attributes:
        self.Vg_leaves = []
        self.AdSamplingFlag = AdSamplingFlag
        self.adapIter = 1
        self.kde_preSamples = []
        self.kde_currSamples = []
        self.initEliteSamples = []
        self.curr_Ldist = 0
        self.prev_Ldist = 0
        self.trackElites = []
        # Reaching the optimal distribution params:
        # ---kde
        self.kdeOpt_flag = False
        self.kde_eliteSamples = []
        self.KDE_fitSamples = None
        self.KDE_pre_gridProbs = None
        self.kde_enabled = True
        # Elite samples and CE computation att
        self.len_frakX = 0
        self.pre_gridProbs = []
        self.SDF_optFlg = False
        self.N_qSamples = 200
        self.NadaptatoinTrajs = 70
        self.bandwidth_adap = 1.5
        self.rho = 0.3
        self.step_size = 0.3
        self.plot_pdf_kde = True

    def planning(self):
        """
        Main planning loop for LQR-CBF-RRT*.
        Iteratively grows the tree by sampling, steering, rewiring, and optionally using adaptive sampling.
        At the end, extracts and saves the optimal path if found.
        """
        start_time = time.time()
        for k in range(self.iter_max):
            # Sample a random node (uniformly or adaptively)
            node_rand = self.generate_random_node(self.goal_sample_rate)
            # Find the nearest node in the tree to the sampled node
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            # Steer from the nearest node towards the random node using LQR
            node_new = self.LQR_steer(node_near, node_rand)

            # Print progress every 5000 iterations
            if k % 5000 == 0:
                print("rrtStar sampling iterations: ", k)
                elapsed = time.time() - start_time
                print(f"Total elapsed time since planning started: {elapsed:.2f} seconds")
                plt.pause(0.001)

            # If a new node is successfully generated and the path is collision-free
            if node_new and not self.utils.is_collision(node_near, node_new):
                # Find all neighbors within a search radius
                neighbor_index = self.find_near_neighbor(node_new)
                self.vertex.append(node_new)

                # Choose the best parent among neighbors and rewire the tree
                if neighbor_index:
                    self.LQR_choose_parent(node_new, neighbor_index)
                    self.rewire(node_new, neighbor_index)

            # Adaptive sampling: try to extend to the goal region using elite samples
            if self.AdSamplingFlag:
                if node_new is None:
                    continue
                r_num = np.random.uniform(0, 1) > 0.5
                if r_num:
                    s_node_dap, g_node_adap = self.s_goal, node_new
                else:
                    s_node_dap, g_node_adap = node_new, self.s_goal

                g_node = self.LQR_steer(s_node_dap, g_node_adap, exact_steering=True)
                if g_node is not None:
                    # If the new node is close enough to the goal, add to elite leaves
                    if (
                        np.linalg.norm(
                            (g_node.x - g_node_adap.x, g_node.y - g_node_adap.y)
                        )
                        < 3.5
                        and not r_num
                    ):
                        self.Vg_leaves.append(g_node)
                    elif (
                        r_num
                        and np.linalg.norm(
                            (g_node.x - g_node_adap.x, g_node.y - g_node_adap.y)
                        )
                        < 3.5
                    ):
                        g_node.cost = g_node.cost + g_node_adap.cost
                        g_node.StateTraj = np.flip(g_node.StateTraj)
                        self.Vg_leaves.append(g_node)

        # After all iterations, search for the best node that can connect to the goal
        index = self.search_goal_parent()

        if index is None:
            print("No path found!")
            return None

        # Extract the optimal path and control sequence
        self.path, self.u_path = self.extract_path(self.vertex[index])

        # Save the path to a .txt file (for use in downstream applications)
        with open("ros2_package_f1_track/src/turtlebot_path_follower/data/final_path.txt", "w") as f:
            for point in reversed(self.path):
                f.write(f"{point[0]}, {point[1]}\n")

        final_cost = self.path_cost(self.path)
        print("optimal distance cost", final_cost)

        # Visualize the final tree and path
        self.plotting.animation(
            self.vertex, self.path, "rrt*, N = " + str(self.iter_max)
        )

    def sample_path(self, wx, wy, u_sequence, step=0.2):
        """
        Interpolates and smooths the path between waypoints using a fixed step size.
        Also computes the cost of the trajectory and the total control effort.

        Args:
            wx (list): List of x-coordinates of waypoints.
            wy (list): List of y-coordinates of waypoints.
            u_sequence (list): List of control inputs (as matrices).
            step (float): Interpolation step size (default: 0.2).

        Returns:
            px (list): Interpolated x-coordinates.
            py (list): Interpolated y-coordinates.
            traj_costs (list): List of segment costs (Euclidean distance).
            u_sequence_cost (float): Total control effort along the path.
        """
        px, py, traj_costs = [], [], []

        # Convert u_sequence from matrix form to list of floats
        u_sequence_list = [
            [u_sequence[i].item(0, 0), u_sequence[i].item(1, 0)]
            for i in range(len(u_sequence))
        ]

        # Interpolate between each pair of waypoints
        for i in range(len(wx) - 1):
            for t in np.arange(0.0, 1.0, step):
                px.append(t * wx[i + 1] + (1.0 - t) * wx[i])
                py.append(t * wy[i + 1] + (1.0 - t) * wy[i])

        dx, dy = np.diff(px), np.diff(py)
        u_sequence_cost = sum([np.linalg.norm(u) for u in u_sequence_list])
        traj_costs = [math.sqrt(idx**2 + idy**2) for (idx, idy) in zip(dx, dy)]

        return px, py, traj_costs, u_sequence_cost

    def LQR_steer(self, node_start, node_goal, exact_steering=False):
        """
        Steers from node_start towards node_goal using LQR-based trajectory generation.
        Optionally, can perform exact steering (for adaptive sampling).

        Args:
            node_start (Node): Starting node.
            node_goal (Node): Target node.
            exact_steering (bool): If True, steer exactly to node_goal (default: False).

        Returns:
            node_new (Node or None): New node reached by LQR steering, or None if not feasible.
        """
        # Compute distance and angle to the target
        dist, theta = self.get_distance_and_angle(node_start, node_goal)
        if not exact_steering:
            # Limit the step size for normal expansion
            dist = min(self.step_len, dist)
            show_animation = False
        else:
            show_animation = False
        # Update the target position for steering
        node_goal.x = node_start.x + dist * math.cos(theta)
        node_goal.y = node_start.y + dist * math.sin(theta)

        # Use LQR planner to generate a trajectory and control sequence
        wx, wy, _, _, u_sequence = self.lqr_planner.lqr_planning(
            node_start.x,
            node_start.y,
            node_goal.x,
            node_goal.y,
            show_animation=show_animation,
            solve_QP=self.solve_QP,
        )
        px, py, traj_cost, u_sequence_cost = self.sample_path(wx, wy, u_sequence)

        if len(wx) == 1:
            return None  # Steering failed or trivial
        node_new = Node((wx[-1], wy[-1]))
        node_new.parent = node_start
        # Calculate cost of the new node (cost-to-come)
        node_new.cost = (
            node_start.cost + sum(abs(c) for c in traj_cost) + u_sequence_cost
        )
        node_new.StateTraj = np.array([px, py])  # For adaptive sampling
        node_new.u_parent_to_current = u_sequence
        return node_new

    def cal_LQR_new_cost(self, node_start, node_goal, cbf_check=True):
        """
        Calculates the cost and feasibility of steering from node_start to node_goal using LQR (and optionally CBF constraints).

        Args:
            node_start (Node): Starting node.
            node_goal (Node): Target node.
            cbf_check (bool): Whether to check CBF constraints (default: True).

        Returns:
            tuple: (cost, can_reach, u_sequence)
                cost (float): Total cost-to-come if feasible, else infinity.
                can_reach (bool): Whether the trajectory is feasible.
                u_sequence (list): Control sequence for the trajectory.
        """
        wx, wy, _, can_reach, u_sequence = self.lqr_planner.lqr_planning(
            node_start.x,
            node_start.y,
            node_goal.x,
            node_goal.y,
            show_animation=False,
            cbf_check=cbf_check,
            solve_QP=self.solve_QP,
        )
        px, py, traj_cost, u_sequence_cost = self.sample_path(wx, wy, u_sequence)
        if wx is None:
            return float("inf"), False
        return (
            node_start.cost + sum(abs(c) for c in traj_cost) + u_sequence_cost,
            can_reach,
            u_sequence,
        )

    def LQR_choose_parent(self, node_new, neighbor_index):
        """
        Chooses the best parent for node_new among its neighbors based on minimum cost-to-come,
        and updates the parent and control sequence accordingly.

        Args:
            node_new (Node): The new node to connect.
            neighbor_index (list): Indices of neighbor nodes in the tree.
        """
        cost = []
        u_neighbor = []  # store u_sequence of neighbor nodes
        for i in neighbor_index:
            # Check if neighbor_node can reach node_new
            _, _, _, can_reach, u_sequence = self.lqr_planner.lqr_planning(
                self.vertex[i].x,
                self.vertex[i].y,
                node_new.x,
                node_new.y,
                show_animation=False,
                solve_QP=self.solve_QP,
            )

            if can_reach and not self.utils.is_collision(
                self.vertex[i], node_new
            ):
                # If feasible and collision-free, compute the cost
                update_cost, _, u_sequence = self.cal_LQR_new_cost(
                    self.vertex[i], node_new
                )
                cost.append(update_cost)
                u_neighbor.append(u_sequence)
            else:
                cost.append(float("inf"))
                u_neighbor.append(None)
        min_cost = min(cost)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        neighbor_index_with_minimum_cost = np.argmin(cost)
        cost_min_index = neighbor_index[neighbor_index_with_minimum_cost]
        node_new.parent = self.vertex[cost_min_index]
        node_new.u_parent_to_current = u_neighbor[neighbor_index_with_minimum_cost]
        # Add the index of node_new to the children of its parent (for rewiring)
        node_new.parent.childrenNodeInds.add(
            len(self.vertex) - 1
        )

    def rewire(self, node_new, neighbor_index):
        """
        Attempts to rewire the tree by connecting neighbor nodes to node_new if it reduces their cost-to-come.
        Updates parent, cost, and control sequence for rewired nodes.

        Args:
            node_new (Node): The new node to potentially become a parent.
            neighbor_index (list): Indices of neighbor nodes in the tree.
        """
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            # Check collision and LQR reachability
            if not self.utils.is_collision(node_new, node_neighbor):
                new_cost, can_rach, u_sequence = self.cal_LQR_new_cost(
                    node_new, node_neighbor
                )

                if can_rach and node_neighbor.cost > new_cost:
                    node_neighbor.parent = node_new
                    node_neighbor.cost = new_cost
                    # Update the control sequence for the rewired node
                    node_neighbor.u_parent_to_current = u_sequence
                    self.updateCosts(node_neighbor)

    def updateCosts(self, node):
        """
        Recursively updates the cost-to-come for all descendants of a given node after rewiring.
        This ensures that cost changes propagate through the subtree.

        Args:
            node (Node): The node whose descendants' costs need updating.
        """
        for ich in node.childrenNodeInds:
            # Update the cost for each child (no need to check CBF again)
            self.vertex[ich].cost = self.cal_LQR_new_cost(
                node, self.vertex[ich], cbf_check=False
            )[
                0
            ]
            self.updateCosts(self.vertex[ich])

    def search_goal_parent(self):
        """
        Searches for the best node in the tree that can connect to the goal region (within goal_len),
        considering only collision-free connections. Returns the index of the best node.

        Returns:
            int or None: Index of the best node to connect to the goal, or None if not found.
        """
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.goal_len]

        if not node_index:
            return None
        # Build cost_list only for nodes that are collision-free
        cost_list = []
        new_node_index = []
        for i in node_index:
            if not self.utils.is_collision(self.vertex[i], self.s_goal):
                cost = dist_list[i] + self.vertex[i].cost
                cost_list.append(cost)
                new_node_index.append(i)

        # Check if there is at least one collision-free connection
        if not cost_list:
            print("[Warning] No collision-free connections to goal found.")
            return None

        # Select the best (minimum cost) node
        best_idx = int(np.argmin(cost_list))
        return new_node_index[best_idx]

    def generate_random_node(self, goal_sample_rate, rce=0.5, md=8):
        """
        Generates a random node for tree expansion.
        Can use uniform sampling, adaptive sampling from elite trajectories, or KDE-based sampling.

        Args:
            goal_sample_rate (float): Probability of sampling the goal directly.
            rce (float): Rare event threshold for adaptive sampling.
            md (int): Parameter for adaptive sampling (unused in this version).

        Returns:
            Node: A randomly sampled node in the state space.
        """
        delta = self.utils.delta
        adap_flag = self.AdSamplingFlag
        u_rand = np.random.uniform(0, 1)
        Vg_leaves = self.Vg_leaves
        if not adap_flag or (
            u_rand > rce and len(Vg_leaves) == 0
        ):
            # Uniform sampling from the workspace
            if np.random.random() > goal_sample_rate:
                return Node(
                    (
                        np.random.uniform(
                            self.x_range[0] + delta, self.x_range[1] - delta
                        ),
                        np.random.uniform(
                            self.y_range[0] + delta, self.y_range[1] - delta
                        ),
                    )
                )
            # With goal_sample_rate probability, sample the goal directly
            return copy.deepcopy(self.s_goal)
        
        # Adaptive sampling from elite trajectories
        elif len(Vg_leaves) != 0 and not self.kdeOpt_flag:
            N_xSmpls = 200
            h = 0.5
            return self.CE_Sample(Vg_leaves, h, self.N_qSamples)
        elif self.kdeOpt_flag:  # Sampling from the optimal SDF (KDE)
            xySmpl = self.KDE_fitSamples.sample()
            return Node((xySmpl[0][0], xySmpl[0][1]))
        else:
            if np.random.random() > goal_sample_rate:
                return Node(
                    (
                        np.random.uniform(
                            self.x_range[0] + delta, self.x_range[1] - delta
                        ),
                        np.random.uniform(
                            self.y_range[0] + delta, self.y_range[1] - delta
                        ),
                    )
                )
            return copy.deepcopy(self.s_goal)

    def CE_Sample(self, Vg_leaves, h, N_qSamples):
        """
        Exploits elite trajectories that reach the goal to adapt the sampling distribution.
        Discretizes elite trajectories to extract samples for KDE-based adaptive sampling.

        Args:
            Vg_leaves (list): List of nodes representing goal-reaching trajectories.
            h (float): Time step for discretization.
            N_qSamples (int): Minimum number of samples required for adaptation.

        Returns:
            Node: Sampled node from the estimated distribution or uniform if insufficient samples.
        """
        if len(Vg_leaves) >= (
            self.adapIter * self.NadaptatoinTrajs
        ):
            frakX = []
            # Find elite trajectories (lowest cost) and discretize them
            Vg_leaves_costList = [vg.cost for vg in Vg_leaves]
            if (self.adapIter + 3) > 5:
                d_factor = 15
            elif self.adapIter > 4:
                d_factor = self.adapIter + 1
            else:
                d_factor = self.adapIter

            q = self.rho / d_factor  # Quantile for elite selection
            cost_rhoth_q = np.quantile(Vg_leaves_costList, q=q)
            elite_Vg_leaves = [vg for vg in Vg_leaves if vg.cost <= cost_rhoth_q]
            if len(elite_Vg_leaves) == 0:
                elite_Vg_leaves = Vg_leaves
            self.trackElites.append((len(elite_Vg_leaves), cost_rhoth_q))
            for vg in elite_Vg_leaves:
                vgcost2come = vg.cost
                # Concatenate the trajectory from root to vg
                traj2vg = np.asarray(vg.StateTraj).T
                node = vg
                while node.parent is not None:
                    node = node.parent
                    if node.cost != 0:
                        ParentTraj = node.StateTraj.T
                        traj2vg = np.concatenate((ParentTraj, traj2vg), axis=0)
                # Discretize the trajectory and extract samples
                tStep_init1 = int(h / self.step_size)
                tStep = 10
                tStep_temp = tStep_init1 + 6
                pi_q_tStep_p = np.array([0, 0])
                while tStep < len(traj2vg[:, 1]):
                    pi_q_tStep = traj2vg[tStep, :]
                    if (
                        np.linalg.norm(
                            (
                                pi_q_tStep[0] - pi_q_tStep_p[0],
                                pi_q_tStep[1] - pi_q_tStep_p[1],
                            )
                        )
                        < 2
                    ):
                        tStep = tStep + tStep_temp
                        continue
                    elite_cddtSample = [
                        pi_q_tStep,
                        vgcost2come,
                    ]
                    frakX.append(elite_cddtSample)
                    tStep = tStep + tStep_temp
                    pi_q_tStep_p = pi_q_tStep

            if self.adapIter == 1:
                frakX.extend(self.initEliteSamples)
            self.len_frakX = len(frakX)
            if len(frakX) == 0:
                ok = 1  # No elite samples found
            x, y = self.CE_KDE_Sampling(frakX)
        else:
            x = None
            y = None
        if x is None or y is None:
            # Fallback to uniform sampling if not enough elite samples
            delta = self.utils.delta
            return Node(
                (
                    np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                    np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta),
                )
            )
        else:
            return Node((x, y))

    def CE_KDE_Sampling(self, frakX):
        """
        Fits a kernel density estimate (KDE) to elite samples and generates a new sample.
        Optionally, visualizes the estimated distribution and checks for convergence using KL divergence.

        Args:
            frakX (list): List of [sample, cost] pairs from elite trajectories.

        Returns:
            tuple: (x, y) coordinates of a sampled point from the KDE.
        """
        frakXarr = np.array(frakX)
        N_samples = len(frakX)
        if len(frakXarr.shape) != 2:
            ok = 1
        costs_arr = frakXarr[:, 1]
        elite_samplesTemp = frakXarr[:, 0]
        elite_samples = [elite_samplesTemp[i] for i in range(len(elite_samplesTemp))]
        elite_samples_arr = np.asarray(elite_samples)
        elite_costs = costs_arr

        # Fit KDE to elite samples
        if self.kde_enabled:
            if self.adapIter % 2 == 0:
                bandwidth = self.bandwidth_adap
            else:
                bandwidth = self.bandwidth_adap
            kde = KernelDensity(kernel="gaussian", bandwidth=bandwidth)
            kde.fit(elite_samples_arr)
            self.adapIter += 1
            xySample = kde.sample()

        # Visualize and check convergence
        if self.kde_enabled:
            x_gridv = np.linspace(self.x_range[0] - 2, self.x_range[1], 60)
            y_gridv = np.linspace(self.y_range[0] - 2, self.y_range[1], 60)
            Xxgrid, Xygrid = np.meshgrid(x_gridv, y_gridv)
            XYgrid_mtx = np.array([Xxgrid.ravel(), Xygrid.ravel()]).T
            grid_probs = np.exp(kde.score_samples(XYgrid_mtx))

            # Check KL divergence for convergence
            if self.adapIter > 2:
                KL_div = self.KLdiv(grid_probs)
                if KL_div < 0.1:
                    self.kdeOpt_flag = True
                    self.KDE_fitSamples = kde

            self.KDE_pre_gridProbs = grid_probs

            # Plot the estimated distribution and elite samples
            if self.plot_pdf_kde:
                CS = plt.contour(
                    Xxgrid, Xygrid, grid_probs.reshape(Xxgrid.shape)
                )
                plt.scatter(elite_samples_arr[:, 0], elite_samples_arr[:, 1])
                plt.show()

        return xySample[0][0], xySample[0][1]

    def KLdiv(self, grid_probs):
        """
        Computes the Kullback-Leibler (KL) divergence between the previous and current KDE grid probabilities.

        Args:
            grid_probs (np.ndarray): Current grid probabilities from KDE.

        Returns:
            float: KL divergence value.
        """
        if self.kde_enabled:
            pre_grid_probs = self.KDE_pre_gridProbs
        else:
            pre_grid_probs = self.pre_gridProbs
        return -sum(
            [
                pre_grid_probs[i] * np.log2(grid_probs[i] / pre_grid_probs[i])
                for i in range(len(pre_grid_probs))
            ]
        )

    def find_near_neighbor(self, node_new):
        """
        Finds all nodes in the tree within a dynamically computed search radius of node_new.
        Only returns nodes that are collision-free with node_new.

        Args:
            node_new (Node): The node to find neighbors for.

        Returns:
            list: Indices of neighbor nodes within the search radius.
        """
        n = len(self.vertex) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [
            math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex
        ]
        dist_table_index = [
            ind
            for ind in range(len(dist_table))
            if dist_table[ind] <= r
            and not self.utils.is_collision(node_new, self.vertex[ind])
        ]
        return dist_table_index

    @staticmethod
    def nearest_neighbor(node_list, n):
        """
        Finds the nearest node in node_list to node n (by Euclidean distance).

        Args:
            node_list (list): List of Node objects.
            n (Node): The node to compare against.

        Returns:
            Node: The nearest node in node_list to n.
        """
        return node_list[
            int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y) for nd in node_list]))
        ]

    def extract_path(self, node_end):
        """
        Extracts the path and control sequence from the start node to node_end by backtracking parents.

        Args:
            node_end (Node): The end node of the path.

        Returns:
            tuple: (path, u_path)
                path (list): List of [x, y] positions from start to goal.
                u_path (list): List of control inputs along the path.
        """
        path = [[self.s_goal.x, self.s_goal.y]]
        u_path = []
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y])
            u_path.extend(node.u_parent_to_current)
            node = node.parent
        path.append([node.x, node.y])

        return path, u_path[::-1]

    @staticmethod
    def path_cost(path):
        """
        Computes the total Euclidean distance of a path.

        Args:
            path (list): List of [x, y] positions.

        Returns:
            float: Total path length.
        """
        src = path[0]
        cost = 0.0

        for i in range(1, len(path)):
            dest = path[i]
            cost += math.hypot(dest[0] - src[0], dest[1] - src[1])
        return cost

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        """
        Computes the Euclidean distance and angle from node_start to node_end.

        Args:
            node_start (Node): Starting node.
            node_end (Node): Target node.

        Returns:
            tuple: (distance, angle in radians)
        """
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def main():
    """
    Entry point for running the LQR-CBF-RRT* planner.
    Initializes the planner with start and goal positions and runs the planning process.
    """
    x_start = (254, -108)  # Starting node
    x_goal = (200, -108)  # Goal node
    rrt_star = LQRrrtStar(x_start, x_goal, 12, 0.001, 80, 50000, AdSamplingFlag=False)
    rrt_star.planning()


if __name__ == "__main__":
    main()