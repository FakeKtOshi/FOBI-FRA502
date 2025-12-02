#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from dynamixel_sdk import *
import numpy as np
from math import pi

# ===============================
# HARDWARE CONFIG
# ===============================
DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 115200

# DYNAMIXEL IDs **IN MDH ORDER**
DXL_IDS = [3, 0, 5, 1, 6, 4, 2]
N = len(DXL_IDS)

JOINT_NAMES = [
    "body",
    "torso",
    "head",
    "shoulder_left",
    "arm_left",
    "shoulder_right",
    "arm_right",
]

# Mechanical limits in radians (same order)
LIMITS_DEG = [
    [-30, 30],   # body
    [-45, 45],   # torso
    [-30, 30],   # head
    [-45, 45],   # shoulder L
    [-30, 30],   # arm L
    [-45, 45],   # shoulder R
    [-30, 30],   # arm R
]
MIN_ANGLE = np.deg2rad([x[0] for x in LIMITS_DEG])
MAX_ANGLE = np.deg2rad([x[1] for x in LIMITS_DEG])

# ===============================
# CONTROL TABLE
# ===============================
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_PWM         = 100
ADDR_PRESENT_PWM      = 124
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_MAX_POS_LIMIT    = 48
ADDR_MIN_POS_LIMIT    = 52

PWM_MODE       = 16
TORQUE_ENABLE  = 1
PWM_LIMIT      = 800

# Conversion constants
VEL_UNIT_TO_RADS = 0.229 * (pi / 30.0)   # raw_vel * 0.229 RPM → rad/s


class DynamixelPWMNode(Node):
    def __init__(self):
        super().__init__("dynamixel_pwm_node")

        # ----------------------
        # SERIAL OPEN
        # ----------------------
        self.port = PortHandler(DEVICENAME)
        self.packet = PacketHandler(2.0)

        if not self.port.openPort():
            raise RuntimeError("❌ Cannot open port")
        self.port.setBaudRate(BAUDRATE)

        self.get_logger().info("✅ Port OK")

        # ----------------------
        # LIMITS & CONVERSION
        # ----------------------
        self.min_raw = np.zeros(N)
        self.max_raw = np.zeros(N)
        self.mid_raw = np.zeros(N)
        self.rad_per_tick = np.zeros(N)

        # ----------------------
        # SETUP EACH MOTOR
        # ----------------------
        for i, dxl in enumerate(DXL_IDS):

            # Set PWM mode
            self.packet.write1ByteTxRx(self.port, dxl, ADDR_OPERATING_MODE, PWM_MODE)
            self.packet.write1ByteTxRx(self.port, dxl, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

            # Read EEPROM limits
            max_val, comm_mx, _ = self.packet.read4ByteTxRx(self.port, dxl, ADDR_MAX_POS_LIMIT)
            min_val, comm_mn, _ = self.packet.read4ByteTxRx(self.port, dxl, ADDR_MIN_POS_LIMIT)

            if comm_mx != COMM_SUCCESS or comm_mn != COMM_SUCCESS:
                min_val, max_val = 0, 4095
                self.get_logger().warn(f"DXL {dxl}: EEPROM read failed, using [0, 4095]")

            self.min_raw[i] = float(min_val)
            self.max_raw[i] = float(max_val)

            # midpoint determines 0 rad
            self.mid_raw[i] = 0.5 * (self.min_raw[i] + self.max_raw[i])

            raw_span = self.max_raw[i] - self.min_raw[i]
            angle_span = MAX_ANGLE[i] - MIN_ANGLE[i]

            if raw_span <= 0:
                raw_span = 4095
                angle_span = 2 * pi

            self.rad_per_tick[i] = angle_span / raw_span

            self.get_logger().info(
                f"DXL {dxl}: raw[{self.min_raw[i]:.0f}, {self.max_raw[i]:.0f}], "
                f"mid={self.mid_raw[i]:.1f}, rad_per_tick={self.rad_per_tick[i]:.6f}"
            )

        # ----------------------
        # ROS INTERFACE
        # ----------------------
        self.sub_pwm = self.create_subscription(
            Float64MultiArray, "/joint_pwm_cmd", self.pwm_callback, 10)

        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)

        self.last_pwm = np.zeros(N)

        self.create_timer(0.01, self.update)  # 100 Hz

        self.get_logger().info("Dynamixel PWM Node ready.")

    # ----------------------
    # Receive PWM
    # ----------------------
    def pwm_callback(self, msg):
        data = np.array(msg.data, dtype=float)
        if data.size != N:
            padded = np.zeros(N)
            padded[: min(N, data.size)] = data[: min(N, data.size)]
            data = padded

        self.last_pwm = np.clip(data, -PWM_LIMIT, PWM_LIMIT)

    # ----------------------
    # Convert DXL raw position → rad
    # ----------------------
    def raw_to_angle(self, i, raw):
        raw = float(raw)

        # Restrict to known limits
        raw = max(self.min_raw[i], min(self.max_raw[i], raw))

        # Convert relative to midpoint
        angle = (raw - self.mid_raw[i]) * self.rad_per_tick[i]

        # Mechanical safety clip
        return float(np.clip(angle, MIN_ANGLE[i], MAX_ANGLE[i]))

    # ----------------------
    # Main loop
    # ----------------------
    def update(self):
        positions = []
        velocities = []

        for i, dxl in enumerate(DXL_IDS):

            # ----- WRITE PWM -----
            pwm = int(self.last_pwm[i])
            if pwm < 0:
                pwm_unsigned = pwm + 65536
            else:
                pwm_unsigned = pwm

            self.packet.write2ByteTxRx(self.port, dxl, ADDR_GOAL_PWM, pwm_unsigned)

            # ----- READ POSITION -----
            pos_raw, comm_pos, _ = self.packet.read4ByteTxRx(self.port, dxl, ADDR_PRESENT_POSITION)
            if comm_pos != COMM_SUCCESS:
                pos_raw = self.mid_raw[i]

            angle = self.raw_to_angle(i, pos_raw)
            positions.append(angle)

            # ----- READ VELOCITY -----
            vel_raw, comm_vel, _ = self.packet.read4ByteTxRx(self.port, dxl, ADDR_PRESENT_VELOCITY)
            if comm_vel != COMM_SUCCESS:
                vel_raw = 0

            # Convert raw unsigned → signed 32-bit
            if vel_raw > 2**31 - 1:
                vel_raw -= 2**32

            vel = vel_raw * VEL_UNIT_TO_RADS
            velocities.append(vel)

            #self.get_logger().info(f"RAW[{i}] = {pos_raw}")
        

        # ----- PUBLISH -----
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = positions
        msg.velocity = velocities

        self.pub_js.publish(msg)

        


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelPWMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
