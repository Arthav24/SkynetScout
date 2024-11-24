# SkynetScout
![CICD Workflow status](https://github.com/Arthav24/SkynetScout/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)  [![codecov](https://codecov.io/gh/Arthav24/SkynetScout/graph/badge.svg?token=0MB8LNC5Y1)](https://codecov.io/gh/Arthav24/SkynetScout) [![License](https://img.shields.io/badge/license-APACHE2.0-blue.svg)](LICENSE)

## Project Contributors
[Amogha Thalihalla Sunil](https://github.com/amoghatsunil),
Navigator
<br>[Anirudh Swarnakar](https://github.com/Arthav24), Driver


## Overview
Welcome to the Inspection Robot repository!

This project focuses on developing an autonomous mobile robot capable of navigating cluttered construction sites and detecting critical anomalies such as cracks, misaligned beams, and hazardous objects. The system combines advanced robotics, computer vision, and AI-based anomaly detection to enhance safety and efficiency in construction site inspections.

### Key Components
- **Autonomous Navigation**
The robot uses SLAM (Simultaneous Localization and Mapping) and advanced path-planning algorithms to navigate dynamic, cluttered environments.
Sensors like LiDAR, RGB cameras are employed for mapping and obstacle avoidance.
- **Anomaly Detection**
AI-based models are used to detect structural cracks, beam misalignments, and hazardous objects. Data from cameras, LiDAR is processed for real-time detection and reporting.
- **Orchestration and Communication**
The skynet_manager package orchestrates all operations, managing inspection workflows, coordinating subsystems, and communicating with site operators and third-party platforms.
A user-friendly interface allows site supervisors to monitor and control the robot.
### **Core ROS2 Packages**
- **anomaly_detection**: Handles anomaly detection, including crack detection, misaligned beam identification, and hazardous object recognition.
- **skynet_manager**: Oversees inspection operations, interfaces with operators, and integrates with external systems for data sharing and control.
- **navigation_stack**: OpenNavigation's [Nav2](https://www.opennav.org/) manages autonomous navigation, mapping, and obstacle avoidance.
Features
- **Real-Time Performance**: Processes sensor data and provides actionable insights in real time.
- **Modular Design**: Easily extendable for additional inspection tasks and use cases.
- **Remote Monitoring**: Allows supervisors to monitor robot operations and receive alerts remotely.

- This repository is designed to accelerate development and deployment of smart inspection solutions, combining cutting-edge robotics and AI technologies to revolutionize construction site operations.

# Setup
```bash
rosdep update
rosdep install --from-paths src --ignore-src -y

sudo apt-get install python3-colcon-clean # this for easy command to clean workspace
```

## UML Diagram - Initial
Class dependency diagram of the proposed design
![image](https://github.com/Arthav24/SkynetScout/blob/sprint1/UML/initial/UML_initial.svg)

## Sequence Diagram - Initial
![image](https://github.com/Arthav24/SkynetScout/blob/sprint1/UML/initial/SequenceDiagram.png)

## Activity Diagram
![image](https://github.com/Arthav24/SkynetScout/blob/sprint1/UML/initial/Activity.svg)

## Dependency Graph
![image](https://github.com/Arthav24/SkynetScout/blob/sprint1/screenshots/depGraph.png)

## ROS module communication - Initial 
![image](https://github.com/Arthav24/SkynetScout/blob/sprint1/UML/initial/ROS_comm.svg)
## Backlog
Product backlog sheet can be found [here](https://umd0-my.sharepoint.com/:x:/g/personal/aniswa_umd_edu/EYVlvxucsS9AoDJi-Hb2Vg8Bc-rh3_DI1Xda5q9So6VFAA?e=bYEpLH&nav=MTVfezMxNTExODU5LUVGMTYtNDQ1OC05QjM0LTIzMzYxNzA3NkQ1NX0)

## Sprint Planning
Using AIP and pair programming project is planned in sprints.
- Sprint 1 11/14/2024 to 11/20/2024
- Sprint 2 11/21/2024 to 11/27/2024
- Sprint 3 11/28/2024 to 12/04/2024

Planning document can be found [here](https://umd0-my.sharepoint.com/:w:/g/personal/aniswa_umd_edu/Ea2nl0-B74VPrtfKL6HL5icBINMij0fw4KHIhCu9YgxoIg?e=M7oarH)

## Proposal Documentation
The proposal documentation for Phase 0 can be found [here](https://umd0-my.sharepoint.com/:b:/g/personal/aniswa_umd_edu/EaGihlp4oVxEorLV7okp_Y8Bgcndar9_CKrkEHib7L6fNw?e=IfWypV)


## Dependencies with licenses
[OpenCV](https://github.com/opencv/opencv) >= 4.5.0 is licensed under the Apache 2 License.
<br> [Eigen](https://github.com/OPM/eigen3/tree/master) is MPL2-licensed.
<br> [ROS2](https://docs.ros.org/en/humble/index.html) is Apache 2 License.
<br> [NAV2](https://docs.nav2.org/) is Apache 2 License.

## Build Instructions

```bash
# Download the source code:
  mkdir -p inspection_ws/src/ 
  git clone https://github.com/Arthav24/SkynetScout.git
  cd SkynetScout
# Configure the project and generate a native build system:
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers console_cohesion+
# Source overlay
source install/setup.bash
```
### Google Coding Style Verification
To check how the written code conforms to the Google C++ style guide, 

```sh
# Install clang-format(ignore if already installed):
  sudo apt install clangd-format
# Self-check Google code style conformity using clang-format:
  clang-format -style=Google -i $( find . -name *.cc | grep -vE -e "^./build/" )
```

### Static Code Analysis
To check the static code analysis of this project
```sh
# Install Cppcheck (ignore if already installed):
  sudo apt install clang-tidy
# Self-check the static code analysis using Cppcheck:
  clang-tidy -p ./ $( find . -name *.cc | grep -v "/build/" )
```
## How to generate module / package dependency graph

``` bash
colcon graph --dot | dot -Tpng -o depGraph.png
open depGraph.png
```

## How to build tests (unit test and integration test)
```bash
colcon clean workspace
colcon build --cmake-args -DCOVERAGE=1 
```

## How to run tests (unit and integration)

```bash
source install/setup.bash
colcon test
```

## How to generate coverage reports after running colcon test

First make sure we have run the unit test already.

```bash
colcon test
```

### Coverage report for `anamoly_detection`:

``` bash
ros2 run anamoly_detection generate_coverage_report.bash
open build/anamoly_detection/test_coverage/index.html
```

### Automate the previous steps and combine both coverage reports

``` bash
./do-tests-and-coverage.bash
```

## How to generate project documentation
``` bash
./do-docs.bash
``` 
