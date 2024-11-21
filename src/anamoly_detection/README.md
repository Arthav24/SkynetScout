# Anomaly Detection

The anomaly_detection package is designed to enable Turtlebot3 to detect construction site anomalies in real-time. This package provides robust tools for crack detection, misaligned beam identification, and hazardous object detection using sensor data such as RGB images, LiDAR point clouds.

Core features include:

Crack Detection: Utilizes deep learning models to identify structural cracks in walls, floors, and beams from high-resolution camera feeds.
Misaligned Beam Detection: Processes 3D LiDAR or depth data to detect deviations in beam alignment compared to pre-defined structural plans.
Hazardous Object Detection: Recognizes potential hazards (e.g., sharp objects, chemical containers) using an object detection model trained on construction-specific datasets.
The package integrates seamlessly with navigation and perception stacks in ROS2 and provides ROS topics and services for anomaly alerts, visualizations, and data logging for further analysis.


## Directory structure

```bash
├── CMakeLists.txt
├── compile_commands.json -> ../../build/anamoly_detection/compile_commands.json
├── include
│   ├── AnomalyDetection.h
│   ├── DetectCrack.h
│   ├── DetectHazardObject.h
│   └── DetectMisAlignedBeams.h
├── launch
│   └── integration_test.launch.yaml
├── LICENSE
├── package.xml
├── README.md
├── src
│   ├── AnomalyDetection.cc
│   ├── DetectCrack.cc
│   ├── DetectHazardObject.cc
│   └── DetectMisAlignedBeams.cc
└── test
    ├── anamoly_detection_integration_test.cc
    └── anamoly_detection_unit_test.cc

```