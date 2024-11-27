# Skynet_manager 
The skynet_manager package serves as the central orchestrator for inspection operations in autonomous construction site robots. It coordinates the execution of inspection tasks, manages sensor and anomaly detection modules, and ensures smooth communication with site operators and third-party systems.

Key functionalities include:

- Inspection Orchestration: Schedules and controls inspection workflows, including navigation, anomaly detection, and data reporting.
- Operator Interface: Provides a real-time interface for site operators to monitor progress, receive anomaly alerts, and issue commands to the robot.
- Third-Party Communication: Facilitates integration with external systems (e.g., construction management software, cloud platforms) for data sharing, reporting, and remote control.

The package leverages ROS2 services, actions, and topics to handle task execution and ensures modular, scalable communication between system components and external entities.

```bash
# run in ws root
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${PWD}/install/skynet_manager/share/skynet_manager/models/
export TURTLEBOT3_MODEL=waffle
source /usr/share/gazebo/setup.bash 
source /usr/share/gazebo-11/setup.bash`

```


## Directory structure

```bash
├── CMakeLists.txt
├── compile_commands.json -> ../../build/skynet_manager/compile_commands.json
├── include
│   └── skynet_manager
│       └── SkynetManager.h
├── launch
│   ├── integration_test.launch.yaml
│   └── turtlebot3_waffle_construction.launch.py
├── LICENSE
├── models
│   ├── beam1
│   ├── beam1_
│   ├── beam1_clone
│   ├── beam2
│   ├── beam2_clone
│   ├── beam2_white
│   ├── crack_sim
│   ├── grey wall
│   ├── unit_cylinder
│   ├── unit_cylinder_0
│   ├── unit_cylinder_1
│   ├── unit_cylinder_1_
│   ├── unit_cylinder_3
│   ├── unit_cylinder_clone
│   └── urban 2 story
├── package.xml
├── README.md
├── src
│   └── SkynetManager.cc
├── test
│   └── skynet_manager_integration_test.cc
├── urdf
└── worlds
    └── construction_site_modv1.world

```