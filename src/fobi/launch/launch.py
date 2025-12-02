from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    pkg = get_package_share_directory("fobi")

    # RViz
    rviz_path = os.path.join(pkg, "config", "display.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_path],
        output="screen"
    )

    # URDF via xacro
    xacro_path = os.path.join(pkg, "robot", "visual", "my-robot.xacro")
    robot_xml = xacro.process_file(xacro_path).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_xml}]
    )

    # Controller node
    controller = Node(
        package="fobi",
        executable="controller.py",
        name="controller_node",
        output="screen"
    )

    # State node **WITH TEST TRAJECTORY PARAMETERS**
    state = Node(
        package="fobi",
        executable="state.py",
        name="state_node",
        output="screen",
        # parameters=[
        #     {"auto_goal": [0.3, 0.2, 0.2, 0.2, 0.2, -0.2, -0.2]},   # <--- TEST VALUES
        #     {"auto_duration": 3.0},                             # 3 seconds
        #     {"joint_limit_deg": 45.0}                           # safety clamp
        # ]
    )

    # Trajectory generator
    trajectory = Node(
        package="fobi",
        executable="trajectory.py",
        name="trajectory_node",
        output="screen"
    )

    # Dynamixel PWM node
    dynamixel = Node(
        package="fobi",
        executable="dynamixel_pwm.py",
        name="dynamixel_pwm_node",
        output="screen"
    )

    # Keyboard
    keyboard = Node(
        package="fobi",
        executable="keyboard.py",
        name="keyboard_node",
        output="screen",
        emulate_tty=True
    )

    return LaunchDescription([
        rviz,
        robot_state_publisher,
        controller,
        state,
        trajectory,
        dynamixel,
        # keyboard  # enable when needed
    ])
