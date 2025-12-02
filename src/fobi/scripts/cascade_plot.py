#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import csv

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class CascadeLogger(Node):
    def __init__(self):
        super().__init__("cascade_logger_with_traj")

        self.N = 7

        # Cached data
        self.data = []
        self.latest_pwm = np.zeros(self.N)

        # Robot actual
        self.prev_qd = None
        self.prev_time = None

        # Reference trajectory
        self.latest_qref = np.zeros(self.N)
        self.latest_qdref = np.zeros(self.N)

        # Subscribers
        self.create_subscription(JointState, "/joint_states", self.cb_js, 10)
        self.create_subscription(Float64MultiArray, "/joint_pwm_cmd", self.cb_pwm, 10)
        self.create_subscription(Float64MultiArray, "/ref_position", self.cb_qref, 10)
        self.create_subscription(Float64MultiArray, "/ref_velocity", self.cb_qdref, 10)

        self.t0 = self.get_clock().now()
        self.get_logger().info("Cascade full logger READY (actual + trajectory)")

    # -------------------------
    def cb_pwm(self, msg):
        arr = np.array(msg.data, dtype=float)
        if arr.size < self.N:
            arr = np.pad(arr, (0, self.N - arr.size), 'constant')
        self.latest_pwm = arr

    def cb_qref(self, msg):
        arr = np.array(msg.data, dtype=float)
        if arr.size < self.N:
            arr = np.pad(arr, (0, self.N - arr.size), 'constant')
        self.latest_qref = arr

    def cb_qdref(self, msg):
        arr = np.array(msg.data, dtype=float)
        if arr.size < self.N:
            arr = np.pad(arr, (0, self.N - arr.size), 'constant')
        self.latest_qdref = arr

    # -------------------------
    def cb_js(self, msg: JointState):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        q = np.array(msg.position[:self.N])
        qd = np.array(msg.velocity[:self.N])

        # acceleration
        if self.prev_qd is None:
            qdd = np.zeros(self.N)
        else:
            dt = t - self.prev_time
            if dt <= 0: dt = 1e-3
            qdd = (qd - self.prev_qd) / dt

        self.prev_qd = qd
        self.prev_time = t

        pwm = self.latest_pwm.copy()

        # reference (q_ref, qd_ref)
        qref = self.latest_qref.copy()
        qdref = self.latest_qdref.copy()

        # Store full row
        row = (
            [t] +
            q.tolist() +
            qd.tolist() +
            qdd.tolist() +
            qref.tolist() +
            qdref.tolist() +
            pwm.tolist()
        )

        self.data.append(row)

    # -------------------------
    def save_csv(self, filename="cascade_full_log.csv"):
        header = ["t"]

        header += [f"q{j}" for j in range(self.N)]
        header += [f"qd{j}" for j in range(self.N)]
        header += [f"qdd{j}" for j in range(self.N)]

        header += [f"qref{j}" for j in range(self.N)]
        header += [f"qdref{j}" for j in range(self.N)]

        header += [f"pwm{j}" for j in range(self.N)]

        self.get_logger().info(f"Saving CSV → {filename}")

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.data)

        self.get_logger().info("CSV saved successfully.")


# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = CascadeLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping... writing CSV...")
    finally:
        node.save_csv("cascade_full_log.csv")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
