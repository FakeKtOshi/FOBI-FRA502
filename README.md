# ROS2-FOBI-Visualization
Krit Leetrakul 6602 (Oshi)

# 🚀 Lab4-6602 Robot Controller (ROS2 Humble)
A unfinished ROS2 control system for a 7-DoF humannoid robot including AUTO cacascade control, TELEOP velocity control, and RViz visualization.

# Project Tree
```
lab4_wspace/
└── src/
    └── fobi/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/launch.py
        ├── config/display.rviz
        ├── meshes/*.stl
        ├── robot/visual/my-robot.xacro
        ├── scripts/
        │   ├── controller.py
        │   ├── dummy_script.py
        │   ├── dynamixel_pwm.py
        │   ├── keyboard.py
        │   ├── state.py
        │   ├── trajectory_logger.py
        │   └── trajectory.py
        └── fobi/dummy_module.py
└── robot_interfaces/
    ├── include/
    ├── src/
    └── srv/
        ├── Controller.srv
        ├── JointTrajectory.srv
        └── State.srv.srv
```

# Install
```
git clone https://github.com/<your_repo>/FOBI.git
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-tf2-ros ros-humble-rviz2 python3-scipy
cd ~/FOBI
colcon build --symlink-install
source install/setup.bash
```

# Run System
# Terminal 1
```
ros2 launch fobi launch.py
```

# Terminal 2
```
ros2 run fobi keyboard.py
```

# Terminal 3 (Choice 1)
```
# If you want to try this mode [TELEOP], then run this command:
ros2 run fobi pwm_mode.py 
```

# Terminal 3 (Choice 2)
```
# If you want to try [AUTo], then run this command with autofeed function (or manually adjust)
ros2 run fobi position_mode.py
ros2 run fobi autofeed.py
```

# Keyboard Commands
* U, J : control both arm rotation
* H, K : control hip rotation
* O, L : control head rotation
```
A : AUTO mode
F : TELEOP

U : +X
J : -X
H : +Y
K : -Y
O : +Z
L : -Z

SPACE : STOP
X : EXIT
```


# Setup RVIz Enviroment
```
- Add RobotModel & TF via By display type
- Click on RobotModel and selected Description Topic to view a model with "/robot_description"
- Then selected "map" from Fixed frame in Global Options, to world
```
# Preview of RVIz 
<img width="1197" height="755" alt="image" src="https://github.com/user-attachments/assets/35baa750-0202-4035-8960-232be2469c36" />

