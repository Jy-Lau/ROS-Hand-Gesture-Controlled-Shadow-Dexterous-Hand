# Description
This ROS1 Noetic project serves as a proof of concept to hand gesture controlled [Shadow Dexterous Hand](https://www.shadowrobot.com/dexterous-hand-series/), allowing users to control the robot's movements using hand gestures.

## Table of Contents
- [Demo](#demo)
- [Installation](#installation)
- [Usage](#usage)
- [External Documentation](#external-documentation)
- [License](#license)

## Demo
<div style="display: flex;">
  <div style="flex: 1; padding-left: 10px;">
    <img src="https://github.com/Jy-Lau/ROS-Hand-Gesture-Controlled-Shadow-Dexterous-Robot/blob/main/hand_robot/doc/gesture-controlled-robot.gif" width="100%">
  </div>
</div>

## Installation
This section provide guidance on how to setup my ROS1 Noetic application.
### Prerequisites
Please install **ROS Noetic Desktop Full** in an Ubuntu VM by following the official installation guide: 🔗 **[ROS Noetic Installation Guide](https://wiki.ros.org/noetic/Installation/Ubuntu)**
You may also use setup Docker on your own.
### Steps
1. Create a ROS workspace:
```bash
  mkdir ros_ws
```
2. Clone the repository:
```bash
  cd ros_ws
  git clone https://github.com/Jy-Lau/ROS-Hand-Gesture-Controlled-Shadow-Dexterous-Hand.git
```
3. Rename the folder:
```bash
  mv ROS-Hand-Gesture-Controlled-Shadow-Dexterous-Hand/ src/
```
4. Clone submodule repositories:
```bash
  cd src
  git submodule init
  git submodule update
```
5. Install ROS dependencies:
```bash
  sudo apt update
  sudo apt install ros-noetic-urdf-geometry-parser
  sudo apt install ros-noetic-moveit
```
6. Install python dependencies:
```bash
  pip3 install cvzone
  pip3 install mediapipe
```
7. Build the package:
```bash
  cd ..
  catkin_make
```
8. Source the workspace:
```bash
  source devel/setup.bash
```

## Usage
This section provide guidance on how to run my ROS1 Noetic application.
### Launching the Nodes
To launch all of the nodes in hand_robot package, run the following command:
```bash
roslaunch hand_robot hand.launch
```

## External Documentation
The official documentation of Shadow Dexterous Hand can be found in the [shadow_dexterous_hand_e_technical_specification.pdf](https://github.com/Jy-Lau/ROS-Hand-Gesture-Controlled-Shadow-Dexterous-Robot/blob/main/hand_robot/doc/shadow_dexterous_hand_e_technical_specification.pdf) file.

## License
This project is licensed under the **Apache 2.0 License** - see the [LICENSE](LICENSE) file for details.
