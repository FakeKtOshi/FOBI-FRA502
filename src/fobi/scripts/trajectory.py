#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from robot_interfaces.srv import JointTrajectory


# ============================================================
# QUINTIC POLYNOMIAL GENERATOR (q0, v0, a0=0) → (qf, vf, af=0)
# ============================================================

def compute_quintic_coeff(q0, v0, a0, qf, vf, af, T):
    """
    Computes a 5th-order polynomial satisfying:
        q(0)  = q0
        q'(0) = v0
        q''(0)= a0
        q(T)  = qf
        q'(T) = vf
        q''(T)= af
    Returns: a[0..5]
    """
    M = np.array([
        [1,    0,      0,        0,        0,         0],
        [0,    1,      0,        0,        0,         0],
        [0,    0,      2,        0,        0,         0],
        [1,    T,      T**2,     T**3,     T**4,      T**5],
        [0,    1,    2*T,     3*T**2,   4*T**3,    5*T**4],
        [0,    0,      2,     6*T,     12*T**2,   20*T**3],
    ], dtype=float)

    b = np.array([q0, v0, a0, qf, vf, af], dtype=float)

    return np.linalg.solve(M, b)


def evaluate_quintic(a, t):
    """Returns q(t), qd(t)."""
    q  = a[0] + a[1]*t + a[2]*t*t + a[3]*t**3 + a[4]*t**4 + a[5]*t**5
    qd =        a[1]   + 2*a[2]*t + 3*a[3]*t*t + 4*a[4]*t**3 + 5*a[5]*t**4
    return q, qd


# ============================================================
# TRAJECTORY SERVICE NODE
# ============================================================

class TrajectoryNode(Node):

    def __init__(self):
        super().__init__("trajectory_node")

        self.declare_parameter("set_frequency", 100.0)
        self.frequency = float(self.get_parameter("set_frequency").value)
        self.dt = 1.0 / self.frequency

        self.srv = self.create_service(
            JointTrajectory,
            "/joint_trajectory",
            self.trajectory_callback
        )

        self.get_logger().info(
            f"[traj] Full Quintic Trajectory Node started (dt={self.dt:.4f}s)"
        )

    # ------------------------------------------------------------
    def _multiarray_2d(self, data_2d: np.ndarray) -> Float64MultiArray:
        """Convert numpy 2D array into ROS2 MultiArray."""
        arr = Float64MultiArray()
        arr.data = data_2d.flatten().tolist()

        n_points, n_joints = data_2d.shape

        arr.layout.dim = [
            MultiArrayDimension(label="time",  size=n_points, stride=n_points * n_joints),
            MultiArrayDimension(label="joint", size=n_joints, stride=n_joints)
        ]
        return arr

    # ------------------------------------------------------------
    def trajectory_callback(self, request, response):

        if request.mode.data != "AUTO":
            self.get_logger().error("[traj] Only AUTO supported.")
            response.inprogress = False
            return response

        # Read data
        q0 = np.array(request.q_start.data, dtype=float)
        qf = np.array(request.q_goal.data, dtype=float)
        v0 = np.array(request.v_start.data, dtype=float)
        vf = np.array(request.v_goal.data, dtype=float)

        if q0.size == 0 or qf.size == 0:
            self.get_logger().error("[traj] Empty q_start or q_goal!")
            response.inprogress = False
            return response

        if q0.size != qf.size:
            self.get_logger().error("[traj] q_start and q_goal mismatch!")
            response.inprogress = False
            return response

        n = q0.size
        T = float(request.duration)

        # Time steps
        num_steps = int(np.ceil(T / self.dt)) + 1
        t_list = np.linspace(0, T, num_steps)

        # Output arrays
        q_traj  = np.zeros((num_steps, n))
        qd_traj = np.zeros((num_steps, n))

        # Full quintic requires acceleration at both ends → set to 0
        a0 = np.zeros(n)
        af = np.zeros(n)

        # Compute polynomial for each joint
        coeffs = []
        for j in range(n):
            a = compute_quintic_coeff(q0[j], v0[j], a0[j],
                                      qf[j], vf[j], af[j], T)
            coeffs.append(a)

        # Evaluate trajectories
        for i, t in enumerate(t_list):
            for j in range(n):
                q, qd = evaluate_quintic(coeffs[j], t)
                q_traj[i, j]  = q
                qd_traj[i, j] = qd

        # Prepare response
        response.inprogress = True
        response.n_points = num_steps
        response.n_joints = n
        response.q_traj   = self._multiarray_2d(q_traj)
        response.qd_traj  = self._multiarray_2d(qd_traj)

        self.get_logger().info(
            f"[traj] FULL QUINTIC generated {num_steps} points (T={T:.2f}s)"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
