# simple_world4ignition_gazebo

## Requirement
ubuntu22.04

ROS2 iron

Ignition Gazebo 6.16.0

## Operating environment
wsl2 in Windows11

## Usage

### Install 
`git clone https://github.com/hirotakasaito/simple_world4ignition_gazebo.git`

### Launch World
`ign gazebo simple_world.sdf`

### Bridge from Ignition Gazebo to ROS2
`ros2 launch ./launch/bridge_ign_ros2.py`

### Operate the robot from the keyboard
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`


