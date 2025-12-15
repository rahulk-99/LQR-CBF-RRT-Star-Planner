"""
LQR (Linear Quadratic Regulator) based motion planning implementation.
This module provides LQR-based trajectory planning with:
1. Single and double integrator dynamics
2. Obstacle avoidance
3. Optimal control synthesis
4. Trajectory optimization

Authors: 
- Tirth Sadaria
- Kunj Golwala
- Rahul Kumar
Date: 16th May 2025
"""

import math
import random
import matplotlib.pyplot as plt
import numpy as np
import scipy
import time

from CBFsteer import CBF_RRT
import env, plotting, utils

SHOW_ANIMATION = False  # Flag to control animation display


class LQRPlanner:
    """
    LQR Planner for velocity control model.
    Implements Linear Quadratic Regulator based motion planning with CBF safety constraints.
    """
    def __init__(self):
        # Time and iteration parameters
        self.MAX_TIME = 100.0  # Maximum simulation time
        self.DT = 0.05  # Time tick
        self.GOAL_DIST = 0.1  # Goal distance threshold
        self.MAX_ITER = 150  # Maximum iterations
        self.EPS = 0.01  # Convergence threshold

        # Initialize system model and LQR controller
        self.A, self.B = self.get_system_model()  # System matrices
        self.K = self.lqr_control(self.A, self.B)  # LQR gain matrix

        # Initialize environment and obstacles
        self.env = env.Env()
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        # Initialize CBF-RRT for safety constraints
        self.cbf_rrt_simulation = CBF_RRT(self.obs_circle)

    def lqr_planning(self, sx, sy, gx, gy, test_LQR=False, show_animation=True, cbf_check=True, solve_QP=False):
        """
        Perform LQR-based motion planning from start to goal position.
        
        Args:
            sx, sy: Start position coordinates
            gx, gy: Goal position coordinates
            test_LQR: Flag to test LQR without CBF constraints
            show_animation: Flag to display animation
            cbf_check: Flag to enable CBF constraint checking
            solve_QP: Flag to solve QP for control input
            
        Returns:
            rx, ry: Planned trajectory coordinates
            error: Distance error at each step
            found_path: Boolean indicating if path was found
            u_sequence: Sequence of control inputs
        """
        # Initialize trajectory and control sequence
        self.cbf_rrt_simulation.set_initial_state(np.array([[sx], [sy]]))
        rx, ry = [sx], [sy]
        error = []
        u_sequence = []
        
        # Initialize state vector (relative to goal)
        x = np.array([sx - gx, sy - gy]).reshape(2, 1)

        found_path = False
        time = 0.0
        
        # Main planning loop
        while time <= self.MAX_TIME:
            time += self.DT

            # Compute LQR control input
            u = self.K @ x

            # Check CBF constraints if enabled
            if cbf_check and not test_LQR and not solve_QP:
                if not self.cbf_rrt_simulation.QP_constraint([x[0, 0] + gx, x[1, 0] + gy], u):
                    print("CBF constraint violated")
                    break

            u_sequence.append(u)

            # Solve QP for control input if enabled
            if solve_QP:
                try:
                    u = self.cbf_rrt_simulation.QP_controller(x, u, model="linear")
                except:
                    print("infeasible")
                    break

            # Update state
            x = self.A @ x + self.B @ u

            # Update trajectory
            rx.append(x[0, 0] + gx)
            ry.append(x[1, 0] + gy)

            # Check goal distance
            d = math.sqrt((gx - rx[-1]) ** 2 + (gy - ry[-1]) ** 2)
            error.append(d)

            if d <= self.GOAL_DIST:
                found_path = True
                break

            # Display animation if enabled
            if show_animation:
                plt.gcf().canvas.mpl_connect(
                    "key_release_event",
                    lambda event: [exit(0) if event.key == "escape" else None],
                )
                plt.plot(sx, sy, "or")
                plt.plot(gx, gy, "ob")
                plt.plot(rx, ry, "-r")
                plt.axis("equal")
                plt.pause(1.0)

        if not found_path:
            return rx, ry, error, found_path, u_sequence

        return rx, ry, error, found_path, u_sequence

    def dlqr(self, A, B, Q, R):
        """
        Solve the discrete time LQR controller.
        Solves the discrete-time algebraic Riccati equation.
        
        Args:
            A: System state matrix
            B: System input matrix
            Q: State cost matrix
            R: Input cost matrix
            
        Returns:
            K: Optimal LQR gain matrix
        """
        # Solve discrete-time algebraic Riccati equation
        P = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
        # Compute LQR gain
        K = np.matrix(scipy.linalg.inv(B.T * P * B + R) * (B.T * P * A))

        # Check closed-loop eigenvalues
        eigVals, eigVecs = scipy.linalg.eig(A - B * K)

        return -K

    def get_system_model(self):
        """
        Get the discrete-time linear system model matrices.
        
        Returns:
            A: State transition matrix
            B: Input matrix
        """
        A = np.matrix([[1, 0], [0, 1]])
        B = np.matrix([[self.DT, 0], [0, self.DT]])
        return A, B

    def lqr_control(self, A, B):
        """
        Compute LQR controller gain matrix.
        
        Args:
            A: State transition matrix
            B: Input matrix
            
        Returns:
            Kopt: Optimal LQR gain matrix
        """
        # Define cost matrices
        Q = np.matrix("1 0; 0 1")  # State cost matrix
        R = np.matrix("0.01 0; 0 0.01")  # Input cost matrix

        Kopt = self.dlqr(A, B, Q, R)
        return Kopt


class LQRPlanner_acceleration:
    """
    LQR Planner for acceleration control model.
    Extends LQR planning to include velocity states and acceleration control inputs.
    """
    def __init__(self):
        # Time and iteration parameters
        self.MAX_TIME = 100.0  # Maximum simulation time
        self.DT = 0.05  # Time tick
        self.GOAL_DIST = 0.1  # Goal distance threshold
        self.MAX_ITER = 150  # Maximum iterations
        self.EPS = 0.01  # Convergence threshold

        # Initialize system model and LQR controller
        self.A, self.B = self.get_system_model()  # System matrices
        self.K = self.lqr_control(self.A, self.B)  # LQR gain matrix

        # Initialize environment and obstacles
        self.env = env.Env()
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        # Initialize CBF-RRT for safety constraints
        self.cbf_rrt_simulation = CBF_RRT(self.obs_circle)

    def lqr_planning(self, sx, sy, svx, svy, gx, gy, gvx, gvy, test_LQR=False, show_animation=True, cbf_check=True):
        """
        Perform LQR-based motion planning with acceleration control.
        
        Args:
            sx, sy: Start position coordinates
            svx, svy: Start velocity components
            gx, gy: Goal position coordinates
            gvx, gvy: Goal velocity components
            test_LQR: Flag to test LQR without CBF constraints
            show_animation: Flag to display animation
            cbf_check: Flag to enable CBF constraint checking
            
        Returns:
            rx, ry: Planned trajectory coordinates
            error: Distance error at each step
            found_path: Boolean indicating if path was found
            u_sequence: Sequence of control inputs
        """
        # Initialize trajectory and control sequence
        self.cbf_rrt_simulation.set_initial_state(np.array([[sx], [sy]]))
        rx, ry, rvx, rvy = [sx], [sy], [svx], [svy]
        error = []
        u_sequence = []

        # Initialize state vector (relative to goal)
        x = np.array([sx - gx, sy - gy, svx - gvx, svy - gvy]).reshape(4, 1)

        found_path = False
        time = 0.0

        # Main planning loop
        while time <= self.MAX_TIME:
            time += self.DT

            # Compute LQR control input
            u = self.K @ x

            # Check CBF constraints if enabled
            if cbf_check and not test_LQR:
                if not self.cbf_rrt_simulation.QP_constraint(
                    [x[0, 0] + gx, x[1, 0] + gy, x[2, 0] + gvx, x[3, 0] + gvy],
                    u,
                    system_type="linear_acceleration_control",
                ):
                    print("violation")
                    break

            u_sequence.append(u)

            # Update state
            x = self.A @ x + self.B @ u

            # Update trajectory
            rx.append(x[0, 0] + gx)
            ry.append(x[1, 0] + gy)

            # Check goal distance
            d = math.sqrt((gx - rx[-1]) ** 2 + (gy - ry[-1]) ** 2)
            error.append(d)

            if d <= self.GOAL_DIST:
                found_path = True
                break

            # Display animation if enabled
            if show_animation:
                plt.gcf().canvas.mpl_connect(
                    "key_release_event",
                    lambda event: [exit(0) if event.key == "escape" else None],
                )
                plt.plot(sx, sy, "or")
                plt.plot(gx, gy, "ob")
                plt.plot(rx, ry, "-r")
                plt.axis("equal")
                plt.pause(1.0)

        if not found_path:
            return rx, ry, error, found_path, u_sequence

        return rx, ry, error, found_path, u_sequence

    def dlqr(self, A, B, Q, R):
        """
        Solve the discrete time LQR controller for acceleration model.
        
        Args:
            A: System state matrix
            B: System input matrix
            Q: State cost matrix
            R: Input cost matrix
            
        Returns:
            K: Optimal LQR gain matrix
        """
        # Solve discrete-time algebraic Riccati equation
        P = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
        # Compute LQR gain
        K = np.matrix(scipy.linalg.inv(B.T * P * B + R) * (B.T * P * A))

        # Check closed-loop eigenvalues
        eigVals, eigVecs = scipy.linalg.eig(A - B * K)

        return -K

    def get_system_model(self):
        """
        Get the discrete-time linear system model matrices for acceleration control.
        
        Returns:
            A: State transition matrix
            B: Input matrix
        """
        A = np.matrix(
            [[1, 0, self.DT, 0], [0, 1, 0, self.DT], [0, 0, 1, 0], [0, 0, 0, 1]]
        )
        B = np.matrix([[0, 0], [0, 0], [self.DT, 0], [0, self.DT]])
        return A, B

    def lqr_control(self, A, B):
        """
        Compute LQR controller gain matrix for acceleration control.
        
        Args:
            A: State transition matrix
            B: Input matrix
            
        Returns:
            Kopt: Optimal LQR gain matrix
        """
        # Define cost matrices
        Q = np.matrix("1 0 0 0; 0 1 0 0; 0 0 0.1 0; 0 0 0 0.1")  # State cost matrix
        R = np.matrix("0.01 0; 0 0.01")  # Input cost matrix

        Kopt = self.dlqr(A, B, Q, R)
        return Kopt


def main():
    """
    Main function to demonstrate LQR planning.
    Tests both velocity and acceleration control models.
    """
    print(__file__ + " start!!")

    # Test parameters
    ntest = 1  # number of goals to test
    area = 50.0  # sampling area size

    # Select control model
    acceleration_model = True

    # Initialize appropriate planner
    if not acceleration_model:
        lqr_planner = LQRPlanner()
    else:
        lqr_planner = LQRPlanner_acceleration()

    # Run planning tests
    for i in range(ntest):
        start_time = time.time()
        
        # Set start and goal positions
        sx = 6.0
        sy = 6.0
        gx = random.uniform(-area, area)
        gy = random.uniform(-area, area)
        print("goal", gy, gx)

        # Run planning based on selected model
        if not acceleration_model:
            rx, ry, error, foundpath, u_sequence = lqr_planner.lqr_planning(
                sx, sy, gx, gy, test_LQR=True, show_animation=SHOW_ANIMATION
            )
        else:
            svx, svy, gvx, gvy = 0, 0, 0, 0
            rx, ry, error, foundpath, u_sequence = lqr_planner.lqr_planning(
                sx, sy, svx, svy, gx, gy, gvx, gvy,
                test_LQR=True, show_animation=SHOW_ANIMATION,
            )

        # Plot results
        f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
        print("time of running LQR: ", time.time() - start_time)

        # Plot trajectory
        ax1.plot(sx, sy, "or")
        ax1.plot(gx, gy, "ob")
        ax1.plot(rx, ry, "-r")
        ax1.grid()

        # Plot error
        ax2.plot(error, label="errors")
        ax2.legend(loc="upper right")
        ax2.grid()
        plt.show()

        # Show animation if enabled
        if SHOW_ANIMATION:
            plt.plot(sx, sy, "or")
            plt.plot(gx, gy, "ob")
            plt.plot(rx, ry, "-r")
            plt.axis("equal")
            plt.pause(1.0)


if __name__ == "__main__":
    main()
