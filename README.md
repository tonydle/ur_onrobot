# ur_onrobot
<img src=images/ur_onrobot.gif width=30%>

ROS drivers for OnRobot Grippers installed on Universal Robots.
For e-Series robots, the gripper can be controlled via the Tool I/O RS-485 interface.

This repository was inspired by and depends on [Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot).

## Features

- Additional controllers for OnRobot [RG2](https://onrobot.com/en/products/rg2-gripper) / [RG6](https://onrobot.com/en/products/rg6-gripper) via Modbus Serial.
- Complete [ros_control](http://wiki.ros.org/ros_control) implementation.
- Modular URDF descriptions for UR - OnRobot combinations.
- Combined bringup launch files for easy setup.

## Dependencies

- [Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot.git)
- [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git)

## Installation

1. Follow all installation instructions for [Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot.git)
2. Clone this repository into your catkin workspace
```bash
cd catkin_ws/src && git clone https://github.com/tonydle/ur_onrobot.git
```
3. Install the rest of the dependencies with rosdep
```bash
cd ../ && rosdep install --from-paths src --ignore-src -r -y
```
4. Build the workspace
```bash
catkin_make
```

## Hardware Setup
### Using the Tool I/O of the UR robot (RS-485)
1. Use a short cable to connect the OnRobot Quick Changer to the Tool I/O of the UR robot.
2. On the UR Teach Pendant, install the [RS485 Daemon URCap](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap)
  - Download the URCap from [Releases](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/releases)
  - Follow the [URCap Installation Guide](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/blob/master/doc/install_urcap.md)
  - Note: Currently there is a bug where if you have the robotiq_grippers URCap installed, the RS485 URCap cannot run.
    Follow the issue [here](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/issues/9) for updates.
  - Restart robot
3. Setup Tool I/O parameters (Installation -> General -> Tool I/O)

    <img src=images/installation_tool_io.png width=60%>

    - Controlled by: User
    - Communication Interface:
        - Baud Rate: 1M
        - Parity: Even
        - Stop Bits: One
        - RX Idle Chars: 1.5
        - TX Idle Chars: 3.5
    - Tool Output Voltage: 24V
    - Standard Output:
        - Digital Output 0: Sinking (NPN)
        - Digital Output 1: Sinking (NPN)

### Using the OnRobot Compute Box
1. Connect the cable between Compute Box and Tool Changer
2. Connect an Ethernet cable between Compute Box and your computer

## Usage
### Bringup when using the Tool I/O (RS-485) with e-Series robots
```bash
roslaunch ur_onrobot bringup_tool_io.launch robot_model:=[ur3e/ur5e] onrobot_model:=[rg2/rg6] robot_ip:=XXX.XXX.XXX.XXX
```
### Bringup when using the OnRobot Compute Box with CB-Series robots
```bash
roslaunch ur_onrobot bringup_compute_box.launch robot_model:=[ur3/ur5] onrobot_model:=[rg2/rg6] robot_ip:=XXX.XXX.XXX.XXX compute_box_ip:=XXX.XXX.XXX.XXX
```

Note: To create a new robot combination, add and configure a new 'load_urX_onrobot_rgX.launch' launch file in ur_onrobot_description

### Getting gripper joint state (finger_width in m)
```bash
rostopic echo /onrobot/joint_states
```
### Getting UR joint state
```bash
rostopic echo /ur/joint_states
```

### ROS Service Calls
#### Open gripper (std_srvs/Trigger)
```bash
rosservice call /onrobot/open
```
#### Close gripper (std_srvs/Trigger)
```bash
rosservice call /onrobot/close
```
#### Restart power (std_srvs/Trigger) - only works with Compute Box
```bash
rosservice call /onrobot/restart_power
```

### Using GripperCommand Action (default controller) to set the position of 'finger_width'
#### Install actionlib_tools if you haven't already
```bash
sudo apt install ros-noetic-actionlib-tools
```
#### Set the finger_width goal with axclient.py
```bash
rosrun actionlib_tools axclient.py /onrobot/gripper_controller/gripper_cmd
```

### Alternatively: Control the gripper with Joint Position Controller
#### Bringup with joint_position controller
Add the argument `gripper_controller:=joint_position` to either your `bringup_tool_io.launch` or `bringup_compute_box.launch` command

#### Set the finger_width
```bash
rostopic pub /onrobot/joint_position_controller/command std_msgs/Float64 "data: 0.05"
```

## Author

[Tony Le](https://github.com/tonydle)

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
