# iai_tracy

## Building the URDF 
You need the ur_description from here: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/jazzy
(Make sure you are on the correct branch)
or install it with apt
```bash
sudo apt install ros-jazzy-ur-description
```

The repo for the gripper can be found here: https://github.com/maltehue/ros2_robotiq_gripper/tree/iai_dualarm

You can put a file names "COLCON_IGNORE in robotiq_hardware_tests" and robotiq_driver if you want to just build the urdf

The repo for the camera can be found at: https://github.com/realsenseai/realsense-ros/tree/ros2-master
