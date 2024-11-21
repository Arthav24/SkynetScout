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
![image](https://github.com/Sounderya22/Human_Tracker_Team7/blob/main/UML/Phase0/activity_diagram.jpeg)

## Sequence Diagram - Initial
![image](https://github.com/Arthav24/SkynetScout/blob/sprint1/UML/initial/SequenceDiagram.png)

## Activity Diagram
![image](https://github.com/Arthav24/SkynetScout/blob/sprint1/UML/initial/SequenceDiagram.png)

## Dependency Graph
![image](https://github.com/Arthav24/SkynetScout/blob/sprint1/UML/initial/SequenceDiagram.png)

## Backlog
Product backlog sheet can be found [here](https://umd0-my.sharepoint.com/:x:/g/personal/aniswa_umd_edu/EYVlvxucsS9AoDJi-Hb2Vg8Bc-rh3_DI1Xda5q9So6VFAA?e=bYEpLH&nav=MTVfezMxNTExODU5LUVGMTYtNDQ1OC05QjM0LTIzMzYxNzA3NkQ1NX0)

## Sprint Planning
Using AIP and pair programming project is planned in sprints.
- Sprint 1 11/14/2024 to 11/20/2024
- Sprint 2 11/21/2024 to 11/27/2024
- Sprint 3 11/28/2024 to 12/04/2024

Planning document can be found [here](https://umd0-my.sharepoint.com/:w:/g/personal/aniswa_umd_edu/Ea2nl0-B74VPrtfKL6HL5icBINMij0fw4KHIhCu9YgxoIg?e=M7oarH)

DRAFT - DRAFT - DRAFT

![CICD Workflow status](https://github.com/TommyChangUMD/ENPM700-final-project-boilerplate/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/TommyChangUMD/ENPM700-final-project-boilerplate/branch/main/graph/badge.svg)](https://codecov.io/gh/TommyChangUMD/ENPM700-final-project-boilerplate) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

This repo provides a template for setting up:

  - GitHub CI
    - `main` branch runs in a ROS 2 Humble container
  - Codecov badges
  - Colcon workspace structure
  - C++ library, `my_model`, that depends on other system libraries such as OpenCV and rclcpp.
    - The library is *self-contained*
    - In real life, we download source code of third-party modules and often put the modules as-is into our colcon workspace.  Of course, we make sure the licenses are all compatible.
  - ROS 2 package, `my_controller`, depends on a C++ library, `my_model`, which is placed into the same colcon workspace.
  - Establishing package dependency within the colcon workspace.
    - ie. the ROS 2 package, `my_controller`, will not be built before all of its dependent C++ libraries, such as `my_model`, are built first.
  - Multiple subscriptions within a ROS2 node all listening to the same topic.
    - Only one callback function is needed.
    - More efficient than to have N callback functions.
    - More efficient than to have N ROS nodes.
  - ROS2 C++ unit test and integration test.
  - Doxygen setup
  - ROS2 launch file
  - Bash scripts that can be invoked by the `ros2 run ...` command

This software uses the **Model-View-Controller** architecture. 
  - **Model** = `my_model` module (see [src/my_model/README.md](src/my_model/README.md))
  - **Controler** = `my_controller` module  (see [src/my_controller/README.md](src/my_controller/README.md))
  - **View** = Gazebo

## How to generate module / package dependency graph

``` bash
colcon graph --dot | dot -Tpng -o depGraph.png
open depGraph.png
```
[<img src=screenshots/depGraph.png
    width="20%" 
    style="display: block; margin: 0 auto"
    />](screenshots/depGraph.png)



## How to build and run demo

First, make sure we install the catch2 ROS2 package.
```bash
$ source /opt/ros/humble/setup.bash  # if needed
$ sudo apt install ros-${ROS_DISTRO}-catch-ros2
```
Now, we can build our system:
```bash
rm -rf build/ install/
colcon build 

```
And finally, run the demo:

```bash
source install/setup.bash
ros2 launch my_controller run_demo.launch.yaml
```
example output:

```
[INFO] [launch]: All log files can be found below /home/tchang/.ros/log/2024-11-14-01-15-43-657639-tchang-IdeaPad-3-17ABA7-2012192
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [talker-1]: process started with pid [2012193]
[INFO] [listener-2]: process started with pid [2012195]
[listener-2] Calling OpenCV function
[listener-2] [INFO] [1731564943.913295525] [my_model]: Calling ROS function
[talker-1] Calling OpenCV function
[talker-1] [INFO] [1731564944.413417817] [my_model]: Calling ROS function
[talker-1] [INFO] [1731564944.413485842] [talker]: Publishing: 101 Hello, world! 0
[listener-2] [INFO] [1731564944.413881423] [listener]: subName=subscription0, I heard : 'Hello, world! 0'
[listener-2] [INFO] [1731564944.413998198] [listener]: subName=subscription1, I heard : 'Hello, world! 0'
[listener-2] [INFO] [1731564944.414027042] [listener]: subName=subscription2, I heard : 'Hello, world! 0'
[listener-2] [INFO] [1731564944.414057074] [listener]: subName=subscription3, I heard : 'Hello, world! 0'
[listener-2] [INFO] [1731564944.414086617] [listener]: subName=subscription4, I heard : 'Hello, world! 0'
[talker-1] Calling OpenCV function
[talker-1] [INFO] [1731564944.911201194] [my_model]: Calling ROS function
[talker-1] [INFO] [1731564944.911250432] [talker]: Publishing: 102 Hello, world! 1
[listener-2] [INFO] [1731564944.911475601] [listener]: subName=subscription0, I heard : 'Hello, world! 1'
[listener-2] [INFO] [1731564944.911539017] [listener]: subName=subscription2, I heard : 'Hello, world! 1'
[listener-2] [INFO] [1731564944.911562413] [listener]: subName=subscription3, I heard : 'Hello, world! 1'
[listener-2] [INFO] [1731564944.911591467] [listener]: subName=subscription4, I heard : 'Hello, world! 1'
[listener-2] [INFO] [1731564944.911633512] [listener]: subName=subscription1, I heard : 'Hello, world! 1'
[talker-1] Calling OpenCV function
[talker-1] [INFO] [1731564945.411270616] [my_model]: Calling ROS function
[talker-1] [INFO] [1731564945.411329702] [talker]: Publishing: 103 Hello, world! 2

```

## How to build tests (unit test and integration test)
We want to run tests with code coverage.  Therefore, we need to enable the code coverage option.

```bash
rm -rf build/ install/
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

### Coverage report for `my_controller`:

``` bash
ros2 run my_controller generate_coverage_report.bash
open build/my_controller/test_coverage/index.html
```

### Coverage report for `my_model`:

``` bash
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select my_model \
       --cmake-target "test_coverage" \
       --cmake-arg -DUNIT_TEST_ALREADY_RUN=1
open build/my_model/test_coverage/index.html
```

### Automate the previous steps and combine both coverage reports

``` bash
./do-tests-and-coverage.bash
```

## How to generate project documentation
``` bash
./do-docs.bash
```

## How to use GitHub CI to upload coverage report to Codecov

There is already a `.github/workflows/run-unit-test-and-upload-codecov.yml` file provided.  But we still need to create a codecov account.

Follow the similar instruction provided in the cpp-boilerplate-v2 repo:

  https://github.com/TommyChangUMD/cpp-boilerplate-v2?tab=readme-ov-file#how-to-use-github-ci-to-upload-coverage-report-to-codecov
  
