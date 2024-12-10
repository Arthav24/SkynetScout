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
├── compile_commands.json -> ../../build/anomaly_detection/compile_commands.json
├── include
│   ├── AnomalyBase.h
│   ├── AnomalyDetection.h
│   ├── DetectCrack.h
│   ├── DetectHazardObject.h
│   └── DetectMisAlignedBeams.h
├── launch
│   ├── anomaly_detection.launch.py
│   └── integration_test.launch.yaml
├── LICENSE
├── package.xml
├── README.md
├── scripts
│   └── generate_coverage_report.bash
├── src
│   ├── AnomalyDetection.cc
│   ├── anomaly_detection_node.cc
│   ├── DetectCrack.cc
│   ├── DetectHazardObject.cc
│   └── DetectMisAlignedBeams.cc
└── test
    ├── anomaly_detection_integration_test.cc
    ├── anomaly_detection_unit_test.cc
    └── test_images
        ├── cracks.png
        ├── misaligned.png
        └── noCrack.png
```