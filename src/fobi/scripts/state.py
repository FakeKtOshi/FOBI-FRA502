#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from robot_interfaces.srv import State, Controller, JointTrajectory


LIMITS_DEG = [
    [-30.0, 30.0],
    [-45.0, 45.0],
    [-30.0, 30.0],
    [-45.0, 45.0],
    [-30.0, 30.0],
    [-45.0, 45.0],
    [-30.0, 30.0],
]


class StateNode(Node):
    def __init__(self):
        super().__init__("state_node")

        # ---------------------------------------------------------
        #       Publish robot state at 100 Hz
        # ---------------------------------------------------------
        self.declare_parameter("set_frequency", 100.0)
        freq = float(self.get_parameter("set_frequency").value)
        self.create_timer(1.0 / freq, self.timer_callback)

        self.robot_state_pub = self.create_publisher(String, "/robot_state", 10)

        # IMPORTANT FIX:
        # State is no longer stuck at "IDLE"
        self.robot_state = "IDLE"

        # ---------------------------------------------------------
        #       Joint state storage for AUTO start point
        # ---------------------------------------------------------
        self.N_JOINTS = 7
        self.q_current = np.zeros(self.N_JOINTS)
        self.have_joint_states = False

        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        # Test goal (you can change this)
        self.qf = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]) # target joint angles in rad
        self.v0 = np.zeros(self.N_JOINTS)
        self.vf = np.zeros(self.N_JOINTS)
        #self.vf = np.array([0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4])
        self.duration = 3.0

        # ---------------------------------------------------------
        #       Service Clients
        # ---------------------------------------------------------
        self.traj_client = self.create_client(JointTrajectory, "/joint_trajectory")
        self.controller_client = self.create_client(Controller, "/controller_service")

        self.get_logger().info("Waiting for /joint_trajectory ...")
        self.traj_client.wait_for_service()

        self.get_logger().info("Waiting for /controller_service ...")
        self.controller_client.wait_for_service()

        # ---------------------------------------------------------
        #       State Service (main input from keyboard)
        # ---------------------------------------------------------
        self.state_service = self.create_service(
            State,
            "/state_service",
            self.state_service_callback
        )

        self.get_logger().info("state_node started")


    # ============================================================
    # Joint state feedback → needed to get q_start
    # ============================================================
    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= self.N_JOINTS:
            self.q_current[:] = np.array(msg.position[:self.N_JOINTS])
            self.have_joint_states = True


    # ============================================================
    # AUTO: send trajectory request
    # ============================================================
    def request_auto_trajectory(self):
        if not self.have_joint_states:
            self.get_logger().error("[AUTO] No joint_states yet → cannot get q0")
            return

        req = JointTrajectory.Request()
        req.mode = String()
        req.mode.data = "AUTO"
        req.q_start.data = self.q_current.tolist()
        req.q_goal.data  = self.qf.tolist()
        req.v_start.data = self.v0.tolist()
        req.v_goal.data  = self.vf.tolist()
        req.duration = float(self.duration)

        future = self.traj_client.call_async(req)
        future.add_done_callback(self.traj_callback)


    # ============================================================
    # AUTO trajectory callback → send to controller
    # ============================================================
    def traj_callback(self, future):
        try:
            res = future.result()
        except:
            self.get_logger().error("[AUTO] Trajectory generator failed")
            self.robot_state = "IDLE"
            return

        if not res.inprogress:
            self.get_logger().error("[AUTO] Trajectory generation error")
            self.robot_state = "IDLE"
            return

        # Build controller request
        req = Controller.Request()
        req.mode = String()
        req.mode.data = "AUTO"
        req.q_traj = res.q_traj
        req.qd_traj = res.qd_traj
        req.n_points = res.n_points
        req.n_joints = res.n_joints
        req.position = Point()

        self.controller_client.call_async(req)

        # KEEP STATE AS "AUTO"
        self.robot_state = "AUTO"


    # ============================================================
    # STATE SERVICE CALLBACK
    # ============================================================
        # ============================================================
    # STATE SERVICE CALLBACK
    # ============================================================
    def state_service_callback(self, request, response):
        """
        Accepts a State request and updates the local state_node state.
        This function supports two ways to request teleop:
          1) request.state.data == "TELEOP" and request.button.data == "F" / "G"
          2) request.state.data == "TELEOP_F" or "TELEOP_G"
        In all cases we send a Controller request with exact modes:
          "AUTO", "IK", "TELEOP_F", "TELEOP_G"
        """
        new_state = request.state.data
        button = request.button.data

        self.get_logger().info(f"[STATE] {self.robot_state} → {new_state}")

        # -----------------------
        # AUTO
        # -----------------------
        if new_state == "AUTO":
            self.robot_state = "AUTO"
            self.request_auto_trajectory()
            response.inprogress = True

        # -----------------------
        # IK
        # -----------------------
        elif new_state == "IK":
            self.robot_state = "IK"
            xyz = [request.position.x, request.position.y, request.position.z]
            # forward IK request to controller
            req = Controller.Request()
            req.mode = String()
            req.mode.data = "IK"
            req.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            self.controller_client.call_async(req)
            response.inprogress = True

        # -----------------------
        # TELEOP (two accepted formats)
        # -----------------------
        elif new_state == "TELEOP" or new_state.startswith("TELEOP"):
            # CASE A: state == "TELEOP" with button specifying F/G
            if new_state == "TELEOP":
                if button == "F":
                    chosen = "TELEOP_F"
                elif button == "G":
                    chosen = "TELEOP_G"
                else:
                    # default fallback: plain TELEOP (no specific sub-mode)
                    chosen = "TELEOP"
            else:
                # CASE B: state already contains the sub-mode ("TELEOP_F" or "TELEOP_G")
                chosen = new_state

            # Update local robot_state and call controller only for sub-modes
            if chosen == "TELEOP_F" or chosen == "TELEOP_G":
                self.robot_state = chosen
                # call controller service with explicit TELEOP_F/TELEOP_G
                self.call_controller_teleop(chosen)
                response.inprogress = True
            else:
                # If user asked plain TELEOP (no F/G) we set local state but do not call controller
                self.robot_state = "TELEOP"
                response.inprogress = True

        # -----------------------
        # Unknown
        # -----------------------
        else:
            self.get_logger().warn(f"Unknown state: {new_state}")
            response.inprogress = False

        # write back the current readback
        response.mode_readback.data = self.robot_state
        return response

        return response


    # Helper for TELEOP
    def call_controller_teleop(self, mode_str):
        req = Controller.Request()
        req.mode = String()
        req.mode.data = mode_str
        req.position = Point()
        self.controller_client.call_async(req)


    # ============================================================
    # Timer → publish the CURRENT robot state
    # ============================================================
    def timer_callback(self):
        msg = String()
        msg.data = self.robot_state
        self.robot_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
