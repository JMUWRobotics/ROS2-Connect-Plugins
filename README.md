# ROS2 Connect Plugins

ROS2 Connect Plugins implements additionally and dynamically loadable plugins for [ROS2 Connect](https://github.com/JMUWRobotics/ROS2-Connect).

These plugins include:
  - Service & Action Server
  - Service & Action Clients
  - Authentication Mechanisms

The source code published here contains an example implementation of a service server & client for adding two integers and an action server & client for calculating the Fibonacci sequence.

Please see the documentation of [ROS2 Connect](https://github.com/JMUWRobotics/ROS2-Connect) for further insights.

## Build

ROS2 Connect Plugins is implemented as a ROS2 package and organized as a colcon project.
It can therefore be built using colcon and compiles (tested) with ROS2 Jazzy Jalisco and later version.

Dependencies:
  - `colcon`
  - `ament_cmake`
  - `ament_cmake_ros`
  - `rclcpp`
  - `rclcpp_action`
  - `pluginlib`
  - [`connect`](https://github.com/JMUWRobotics/ROS2-Connect)
  <br><br>
  - `example_interfaces` (only for this exemplary source code)

## License

This project is licensed under the **Mozilla Public License 2.0 (MPL-2.0)**. 
The MPL-2.0 is a "file-level copyleft" license that balances the needs of open-source and proprietary software.

Please see the [LICENSE](/LICENSE) file for the full license text.

[![License: MPL 2.0](https://img.shields.io/badge/License-MPL_2.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)