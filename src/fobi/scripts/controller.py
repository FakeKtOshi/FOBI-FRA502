#!/usr/bin/python3
import rclpy
from rclpy.node import Node

import numpy as np
from math import pi

from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState

from spatialmath import SE3
import roboticstoolbox as rtb

from robot_interfaces.srv import Controller


# Mechanical limits (deg), MDH order:
LIMITS_DEG = [
    [-30.0, 30.0],   # body
    [-45.0, 45.0],   # torso
    [-30.0, 30.0],   # head
    [-45.0, 45.0],   # shoulder_left
    [-30.0, 30.0],   # arm_left
    [-45.0, 45.0],   # shoulder_right
    [-30.0, 30.0],   # arm_right
]


class ControllerNode(Node):
    """
    Cascade controller:
      AUTO/IK: q_ref -> PI_pos -> qd_ref -> (FF + PI_vel) -> PWM
      TELEOP: direct PWM on selected joints (no cascade)
    """

    def __init__(self):
        super().__init__("controller_node")

        # ---------------------
        # Timing
        # ---------------------
        self.declare_parameter("set_frequency", 100.0)
        self.frequency = float(self.get_parameter("set_frequency").value)
        self.dt = 1.0 / self.frequency
        self.create_timer(self.dt, self.timer_callback)

        # ---------------------
        # Robot model (RViz / IK only)
        # ---------------------
        L1 = rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.00925)
        L2 = rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.1575)
        L3 = rtb.RevoluteMDH(alpha=pi/2.0, a=0.0, d=0.0)
        L4 = rtb.RevoluteMDH(alpha=-pi/2, a=0.0, d=0.095, offset=pi/4)
        L5 = rtb.RevoluteMDH(alpha=pi/2, a=0.0, d=0.0)
        L6 = rtb.RevoluteMDH(alpha=-pi/2, a=0.0, d=0.095, offset=135*pi/180.0)
        L7 = rtb.RevoluteMDH(alpha=pi/2, a=0.0, d=0.0)

        self.robot = rtb.DHRobot([L1, L2, L3, L4, L5, L6, L7], tool=SE3())
        self.N = 7

        # ---------------------
        # Gains (cascade)
        # ---------------------
        # Outer: position PI -> velocity reference
        self.Kp_pos = 1.0   # small for smooth moves (user requested very smooth)
        self.Ki_pos = 0.20

        # Inner: velocity PI -> PWM
        # chosen conservatively for PWM control (user reported PWM=200 -> decently fast)
        self.Kp_vel = 50.0
        self.Ki_vel = 2.0

        # Feedforward: map desired velocity -> PWM
        # Estimate: PWM ≈ Kff * qd (Kff tuned conservatively)
        # (user: PWM=200 => decently fast; we pick Kff ~ 60)
        self.Kff = 60.0

        # ---------------------
        # PWM limits (separate for AUTO and TELEOP)
        # ---------------------
        self.AUTO_PWM_LIMIT = 150.0   # safe for trajectory following
        self.TELEOP_PWM_LIMIT = 800.0 # full range for manual control

        self.pwm_limit = self.AUTO_PWM_LIMIT  # current active limit (switched by mode)

        # ---------------------
        # Limits
        # ---------------------
        limits_rad = np.deg2rad(np.array(LIMITS_DEG, dtype=float))
        self.q_min = limits_rad[:, 0]
        self.q_max = limits_rad[:, 1]

        # ---------------------
        # Velocity caps (safe)
        # ---------------------
        self.v_max_auto = 0.30   # rad/s (smooth, user asked)
        self.v_max_default = 1.0 # fallback (not used for AUTO)

        # ---------------------
        # Teleop
        # ---------------------
        self.tele_speed = 0.0
        self.teleop_timeout = 0.2
        self.last_cmd_time = self.get_clock().now()

        # ---------------------
        # State
        # ---------------------
        self.robot_state = "IDLE"
        self.q_meas = np.zeros(self.N)
        self.qd_meas = np.zeros(self.N)
        self.have_feedback = False

        self.q_ref = np.zeros(self.N)
        self.qd_ref = np.zeros(self.N)

        self.int_pos_err = np.zeros(self.N)
        self.int_v_err = np.zeros(self.N)

        # Trajectory
        self.traj_q = None
        self.traj_qd = None
        self.traj_idx = 0
        self.traj_active = False

        # publish reference for logger / plotting
        self.ref_pos_pub = self.create_publisher(Float64MultiArray, "/ref_position", 10)
        self.ref_vel_pub = self.create_publisher(Float64MultiArray, "/ref_velocity", 10)


        # ---------------------
        # ROS interface
        # ---------------------
        self.pwm_pub = self.create_publisher(Float64MultiArray, "/joint_pwm_cmd", 10)
        self.endeffector_pub = self.create_publisher(PoseStamped, "/end_effector", 10)

        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        self.create_service(Controller, "/controller_service", self.controller_service_callback)

        # debug counters
        self.no_feedback_counter = 0

        self.get_logger().info("controller_node (cascade + FF) READY")

    # ---------------------
    # Service: accept q_traj and optionally qd_traj
    # ---------------------
    def controller_service_callback(self, request, response):
        mode = request.mode.data

        self.get_logger().info(f"[SERVICE] /controller_service called: mode={mode}")

        # AUTO
        if mode == "AUTO":
            try:
                q_flat = np.array(request.q_traj.data, dtype=float)
                q_all = q_flat.reshape((request.n_points, request.n_joints))
            except Exception as e:
                self.get_logger().error(f"[AUTO] q_traj reshape failed: {e}")
                response.inprogress = False
                return response

            qd_all = None
            try:
                if len(request.qd_traj.data) > 0:
                    qd_flat = np.array(request.qd_traj.data, dtype=float)
                    qd_all = qd_flat.reshape((request.n_points, request.n_joints))
            except Exception as e:
                self.get_logger().warn(f"[AUTO] qd_traj present but reshape failed: {e}")
                qd_all = None

            if request.n_joints != self.N:
                self.get_logger().error(f"[AUTO] Expected {self.N} joints, got {request.n_joints}")
                response.inprogress = False
                return response

            # clip positions to mechanical limits
            q_all = np.clip(q_all, self.q_min, self.q_max)

            # store
            self.traj_q = q_all
            if qd_all is not None:
                self.traj_qd = qd_all
            else:
                dt = float(request.duration) / max(1, (request.n_points - 1))
                self.traj_qd = np.gradient(self.traj_q, dt, axis=0)

            self.traj_idx = 0
            self.traj_active = True
            self.robot_state = "AUTO"
            self.pwm_limit = self.AUTO_PWM_LIMIT  # switch to safe AUTO limit

            # reset integrators
            self.int_pos_err[:] = 0.0
            self.int_v_err[:] = 0.0

            self.get_logger().info(f"[AUTO] Loaded {len(self.traj_q)} points (qd provided={qd_all is not None})")
            response.inprogress = True
            return response

        # IK
        elif mode == "IK":
            T = SE3(request.position.x, request.position.y, request.position.z)
            sol = self.robot.ikine_LM(T, mask=[1, 1, 1, 0, 0, 0], joint_limits=True)
            if not sol.success:
                self.get_logger().error("[IK] IK failed")
                response.inprogress = False
                return response

            # IK should also be safe
            self.pwm_limit = self.AUTO_PWM_LIMIT

            self.get_logger().info(f"[IK] Entered IK target: {T.t}")
            response.inprogress = True
            return response

        # TELEOP
        elif mode in ["TELEOP_F", "TELEOP_G"]:
            self.robot_state = mode
            self.traj_active = False

            # TELEOP allows full PWM
            self.pwm_limit = self.TELEOP_PWM_LIMIT

            self.get_logger().info(f"[TELEOP] Enter {mode}, pose frozen")
            response.inprogress = True
            return response

        # unknown
        response.inprogress = False
        return response

    # ---------------------
    # Subscribers
    # ---------------------
    def cmd_vel_callback(self, msg: Twist):
        # keyboard publishes linear.x as teleop speed
        self.tele_speed_x = msg.linear.x
        self.tele_speed_y = msg.linear.y
        self.tele_speed_z = msg.linear.z

        self.last_cmd_time = self.get_clock().now()

    def joint_state_callback(self, msg: JointState):
        if not msg.position:
            return
        n = min(self.N, len(msg.position))
        self.q_meas[:n] = np.array(msg.position[:n], dtype=float)
        if len(msg.velocity) >= n:
            self.qd_meas[:n] = np.array(msg.velocity[:n], dtype=float)
        self.have_feedback = True

    
    # ---------------------
    # Timer: cascade control loop
    # ---------------------
    def timer_callback(self):
        if not self.have_feedback:
            # warn once per second
            self.no_feedback_counter += 1
            if self.no_feedback_counter >= int(max(1, self.frequency)):
                self.get_logger().warn("[CONTROLLER] No joint feedback (/joint_states) received yet.")
                self.no_feedback_counter = 0
            return
        else:
            self.no_feedback_counter = 0

        # --- 1) set q_ref/qd_ref from trajectory or mode ---
        if self.robot_state == "AUTO" and self.traj_active:
            if self.traj_idx < len(self.traj_q):
                self.q_ref[:] = self.traj_q[self.traj_idx, :]
                self.qd_ref[:] = self.traj_qd[self.traj_idx, :]
                self.traj_idx += 1
            else:
                self.traj_active = False
                self.get_logger().info("[AUTO] Done")

        elif self.robot_state == "IK":
            pwm_vec = np.zeros(self.N)


            pass
        
        elif self.robot_state in ["TELEOP_F", "TELEOP_G"]:
            # Direct PWM teleop (simple, clean, no cascade)
            pwm_vec = np.zeros(self.N)
            time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

            if time_since_cmd <= self.teleop_timeout:
                tele_pwm_scale = 300.0  # tunable

                # ---------------------------
                # READ INPUTS FROM /cmd_vel
                # ---------------------------
                v_x = self.tele_speed_x           # U/J => shoulders
                v_y = self.tele_speed_y  # H/K => head
                v_z = self.tele_speed_z  # O/L => body

                # scale to PWM
                pwm_x = np.clip(v_x * tele_pwm_scale, -self.pwm_limit, self.pwm_limit)
                pwm_y = np.clip(v_y * tele_pwm_scale, -self.pwm_limit, self.pwm_limit)
                pwm_z = np.clip(v_z * tele_pwm_scale, -self.pwm_limit, self.pwm_limit)

                # ---------------------------
                # JOINT MAPPING
                # ---------------------------
                BODY = 0
                HEAD = 2
                LEFT_SHOULDER = 3
                RIGHT_SHOULDER = 5

                # Body (O/L)
                pwm_vec[BODY] = pwm_z

                # Head (H/K)
                pwm_vec[HEAD] = pwm_y

                # Shoulders (U/J)
                pwm_vec[LEFT_SHOULDER] = -pwm_x   # left inverted
                pwm_vec[RIGHT_SHOULDER] = pwm_x   # right normal

            else:
                pwm_vec[:] = 0.0

            # publish PWM and skip cascade completely
            # publish current q_ref / qd_ref so logger can record trajectory reference
            msg_qref = Float64MultiArray()
            msg_qref.data = self.q_ref.astype(float).tolist()
            self.ref_pos_pub.publish(msg_qref)

            msg_qdref = Float64MultiArray()
            msg_qdref.data = self.qd_ref.astype(float).tolist()
            self.ref_vel_pub.publish(msg_qdref)

            msg = Float64MultiArray()
            msg.data = pwm_vec.tolist()
            self.pwm_pub.publish(msg)
            return

        else:
            # IDLE
            self.q_ref[:] = self.q_meas[:]
            self.qd_ref[:] = 0.0

        # clamp q_ref
        self.q_ref = np.clip(self.q_ref, self.q_min, self.q_max)

        # --- 2) outer loop: PI on position -> produce a velocity correction ---
        if self.robot_state in ["AUTO", "IK"]:
            pos_err = self.q_ref - self.q_meas
            self.int_pos_err += pos_err * self.dt

            # anti-windup: limit integral
            max_int_pos = (self.v_max_auto) / max(self.Ki_pos, 1e-9)
            self.int_pos_err = np.clip(self.int_pos_err, -max_int_pos, max_int_pos)

            v_cmd = self.Kp_pos * pos_err + self.Ki_pos * self.int_pos_err

            # combine with trajectory velocity reference if present
            self.qd_ref = self.qd_ref + v_cmd

            # clamp velocities to safe v_max_auto for AUTO/IK
            self.qd_ref = np.clip(self.qd_ref, -self.v_max_auto, self.v_max_auto)

        else:
            self.qd_ref[:] = 0.0

        # --- 3) inner loop: VELOCITY PI + FEEDFORWARD -> PWM ---
        # Compute velocity error
        v_err = self.qd_ref - self.qd_meas

        # integrate velocity error
        self.int_v_err += v_err * self.dt

        # anti-windup for velocity integrator (based on pwm_limit)
        max_int_v = self.pwm_limit / max(self.Ki_vel, 1e-9)
        self.int_v_err = np.clip(self.int_v_err, -max_int_v, max_int_v)

        # feedforward term (maps desired velocity -> nominal PWM)
        ff = self.Kff * self.qd_ref

        # PI feedback
        fb = self.Kp_vel * v_err + self.Ki_vel * self.int_v_err

        pwm_vec = ff + fb
        pwm_vec = np.clip(pwm_vec, -self.pwm_limit, self.pwm_limit)

        # publish PWM vector (N elements)
        msg = Float64MultiArray()
        msg.data = pwm_vec.astype(float).tolist()
        self.pwm_pub.publish(msg)

        # --- 4) logging (concise) ---
        j0_pos_err = self.q_ref[0] - self.q_meas[0]
        j0_vel_err = self.qd_ref[0] - self.qd_meas[0]
        j0_pwm = pwm_vec[0]
        self.get_logger().info(
            f"[CASCADE] state={self.robot_state}, q0_ref={self.q_ref[0]:.3f}, q0={self.q_meas[0]:.3f}, "
            f"e_q0={j0_pos_err:.4f}, qd0_ref={self.qd_ref[0]:.3f}, qd0={self.qd_meas[0]:.3f}, e_v0={j0_vel_err:.4f}, pwm0={j0_pwm:.1f}"
        )

    # ===========================
    # MAIN
    # ===========================
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
