cmake_minimum_required(VERSION 3.8)
project(anamoly_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(catch_ros2 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  #   build the integration test
  ###########################################################################
  # 1.) BUILD AND INSTALL THE INTEGRATION TEST NODE (aka Node that performs the test)  ##
  ###########################################################################
  # This is the node in which integration tests occur
  add_executable(AnamolyDetectionTest
          test/anamoly_detection_integration_test.cc
  )
  # The link libraries call this node with "catch_ros2::catch_ros2_with_node_main"
  # to get the default integration test node main function
  target_link_libraries(AnamolyDetectionTest
          catch_ros2::catch_ros2_with_node_main
  )
  ament_target_dependencies(AnamolyDetectionTest
          rclcpp std_srvs std_msgs sensor_msgs tf2 tf2_ros geometry_msgs
  )
  install(TARGETS
          AnamolyDetectionTest
          DESTINATION lib/${PROJECT_NAME}
  )

  ######################################
  ## 2.) ADD INTEGRATION TEST SUPPORT ##
  ######################################
  # This CMake function allows the integration test to be run
  # when running "colcon test".
  catch_ros2_add_integration_test (AnamolyDetection_Test
          LAUNCH_FILE integration_test.launch.yaml
  )
endif()

ament_package()
