# Welcome to 

## Get Started
### Directory Structure

The Aurora ROS2 SDK contains the resources and code you may need during your development process. The directory structure is organized as follows:

| Directory              | Description                               |
| ---------------------- | ----------------------------------------- |
| src                    | Source code                               |
| --slamware_ros_sdk     | Source code of Slamware ROS SDK           |
| --aurora_remote_public | Aurora-related header files and libraries |

### Development Environment

- The SDK is based on Ubuntu 20.04 / 22.04 operating systems and requires the installation of ROS2 packages.

### Hardware Requirements

To use the ROS2 SDK

### Hello World

#### 1. Create workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

#### 2. Install
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

#### 3. Compile

```bash
cd ..
colcon build
```
