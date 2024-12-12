cmake_minimum_required(VERSION 3.8)
project(skynet_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/AnomalyStatus.msg"
        "msg/Anomalies.msg"
        "srv/StartInspection.srv"
        "srv/ReturnToHome.srv"
        DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
