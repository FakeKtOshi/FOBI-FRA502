#!/usr/bin/python3

from fobi.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import State
from geometry_msgs.msg import Twist

import sys
import termios
import tty


# --------------------------------------------------
# Non-blocking 1-char keyboard input
# --------------------------------------------------
def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        c = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return c


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')

        # State service client
        self.cli = self.create_client(State, "/state_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /state_service ...")

        # Teleop velocity publisher
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.speed = 0.8  # m/s
        self.teleop_active = False   # <--- Important

        self.get_logger().info("Keyboard ready!")
        self.get_logger().info("========== CONTROLS ==========")
        self.get_logger().info("  A : AUTO mode")
        self.get_logger().info("  I : IK mode → enter X Y Z")
        self.get_logger().info("  F : TELEOP_F (frame velocity)")
        self.get_logger().info("  G : TELEOP_G (global velocity)")
        self.get_logger().info("")
        self.get_logger().info("  U : +X")
        self.get_logger().info("  J : -X")
        self.get_logger().info("  H : +Y")
        self.get_logger().info("  K : -Y")
        self.get_logger().info("  O : +Z")
        self.get_logger().info("  L : -Z")
        self.get_logger().info("")
        self.get_logger().info("  SPACE : STOP")
        self.get_logger().info("  X : EXIT")
        self.get_logger().info("================================")

    # --------------------------------------------------
    def send_state(self, state, button="", xyz=None):
        req = State.Request()
        req.state.data = state
        req.button.data = button

        if xyz is not None:
            req.position.x = float(xyz[0])
            req.position.y = float(xyz[1])
            req.position.z = float(xyz[2])

        return self.cli.call_async(req)

    # --------------------------------------------------
    def publish_velocity(self, x, y, z):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = float(z)
        self.vel_pub.publish(msg)

    # --------------------------------------------------
    def loop(self):
        while rclpy.ok():
            key = getch().lower()

            # EXIT
            if key == "x":
                self.get_logger().info("Exit keyboard.")
                break

            # ======================
            # STATE: AUTO
            # ======================
            if key == "a":
                self.get_logger().info("Request AUTO")
                self.teleop_active = False
                self.send_state("AUTO")

            # ======================
            # STATE: IK
            # ======================
            elif key == "i":
                self.get_logger().info("Enter IK X Y Z:")
                xyz_str = input().split()

                if len(xyz_str) != 3:
                    self.get_logger().error("Invalid input! Use: x y z")
                    continue

                x = float(xyz_str[0])
                y = float(xyz_str[1])
                z = float(xyz_str[2])

                self.get_logger().info(f"Sending IK target → ({x}, {y}, {z})")
                self.teleop_active = False
                self.send_state("IK", xyz=[x, y, z])

            # ======================
            # TELEOP MODE SELECT
            # ======================
            elif key == "f":
                self.get_logger().info("Switch to TELEOP_F")
                self.teleop_active = True
                self.send_state("TELEOP", button="F")

            elif key == "g":
                self.get_logger().info("Switch to TELEOP_G")
                self.teleop_active = True
                self.send_state("TELEOP", button="G")

            # ======================
            # TELEOP MOTION (U/H/J/K/O/L)
            # ======================
            elif self.teleop_active and key == "u":   # +X
                self.publish_velocity(self.speed, 0.0, 0.0)

            elif self.teleop_active and key == "j":   # -X
                self.publish_velocity(-self.speed, 0.0, 0.0)

            elif self.teleop_active and key == "h":   # +Y
                self.publish_velocity(0.0, self.speed, 0.0)

            elif self.teleop_active and key == "k":   # -Y
                self.publish_velocity(0.0, -self.speed, 0.0)

            elif self.teleop_active and key == "o":   # +Z
                self.publish_velocity(0.0, 0.0, self.speed)

            elif self.teleop_active and key == "l":   # -Z
                self.publish_velocity(0.0, 0.0, -self.speed)

            # STOP
            elif key == " ":
                self.get_logger().info("STOP")
                self.publish_velocity(0.0, 0.0, 0.0)

            else:
                self.get_logger().info(f"Unknown key: '{key}'")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
