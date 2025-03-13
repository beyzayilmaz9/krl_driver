# Manual Drive Package for ROS2 Humble (C++)

## Overview
This package provides a manual driving implementation for ROS2 Humble using C++. It allows controlling a robot manually via commands sent through ROS2 topics.

## Features
- ROS2 Humble compatibility
- Manual control through a ROS2 topic
- Modular C++ implementation
- Parameterized settings for customization

## Installation
Ensure you have ROS2 Humble installed on your system. Then, follow these steps:

```bash
cd ~/ros2_ws/src
git clone <repository_url>
cd ~/ros2_ws
colcon build --packages-select <package_name>
source install/setup.bash
```

## Dependencies
This package requires the following dependencies:
- `rclcpp` (ROS2 C++ client library)
- `geometry_msgs` (for sending twist messages)
- `std_msgs` (for standard message types)

Ensure you have them installed:
```bash
sudo apt install ros-humble-rclcpp ros-humble-geometry-msgs ros-humble-std-msgs
```

## Usage
To launch the manual drive node, run:
```bash
ros2 run <package_name> <node_executable>
```

You can control the robot by publishing velocity commands:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

## Configuration
You can modify parameters in the `params/` directory to change the behavior of the manual driving system.

## Files Structure
```
├── include/krl_driver/      # Header files
├── params/                  # Configuration parameters
├── src/                     # Source code
├── CMakeLists.txt           # Build system
├── package.xml              # ROS2 package manifest
├── README.md                # Documentation
```

## License
This package is open-source and available under the MIT License.

## Contributors
- Beyzanur Yılmaz
