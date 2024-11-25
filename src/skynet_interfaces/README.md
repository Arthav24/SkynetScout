# Skynet Interfaces

This package contains all the custom interfaces used in project. Interfaces are of type .msg, .srv and others. 



## Directory structure

```bash
├── CMakeLists.txt
├── include
│   └── skynet_interfaces
├── LICENSE
├── msg
│   ├── Anamolies.msg
│   └── AnamolyStatus.msg
├── package.xml
├── README.md
├── src
├── srv
│   ├── ReturnToHome.srv
│   └── StartInspection.srv
└── test

```

## Structure details

### Anamolies.msg
This message is aggregation of all the anomalies found in the sample frame.
```bash
std_msgs/Header header
AnamolyStatus[] anamolies
```

### AnamolyStatus.msg
This message is published by individual anomaly detection libraries.
```bash
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3
byte CRACK=4
byte MBEAMS=5
byte HOBJECT=6
byte level
string name
string message
byte type
geometry_msgs/Pose position
string location
```

### ReturnToHome.srv
This service is used to call the robot back to home after inspection or abort the mission and come home.
```bash
bool return_to_home
---
bool success
string message
```

### StartInspection.srv
This service is used to start the inspection. 
```bash
int8 inspection_route_no
---
bool success
string message
```