cmake_minimum_required(VERSION 3.8)
project(anamoly_detection)

set(CMAKE_CXX_STANDARD 17)
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

###################################
# find dependencies
###################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(Doxygen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

############################################
# Set up level 2 unit test (i.e, integration test)
############################################
#if(COVERAGE)
#  set(CMAKE_BUILD_TYPE "Debug")
#  set(CMAKE_CXX_FLAGS "--coverage")
#  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
#endif()
if (BUILD_TESTING)
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
    add_executable(anomalyDetectionTest
            test/anamoly_detection_integration_test.cc
    )
    # The link libraries call this node with "catch_ros2::catch_ros2_with_node_main"
    # to get the default integration test node main function
    target_link_libraries(anomalyDetectionTest
            catch_ros2::catch_ros2_with_node_main
    )
    target_include_directories(anomalyDetectionTest PUBLIC
            ${OpenCV_INCLUDE_DIRS}
    )
    ament_target_dependencies(anomalyDetectionTest
            rclcpp std_srvs std_msgs sensor_msgs tf2 tf2_ros geometry_msgs OpenCV
    )
    install(TARGETS
            anomalyDetectionTest
            DESTINATION lib/${PROJECT_NAME}
    )

    ######################################
    ## 2.) ADD INTEGRATION TEST SUPPORT ##
    ######################################
    # This CMake function allows the integration test to be run
    # when running "colcon test".
    catch_ros2_add_integration_test(anomalyDetection_Test
            LAUNCH_FILE integration_test.launch.yaml
    )
    include(CTest)

    ###############
    ## UNIT TEST ##
    ###############
    add_executable(anamoly_detection_unitTests
            test/anamoly_detection_unit_test.cc
    )
    # Linking a unit test executable with catch_ros2::catch_ros2_with_main
    # provides the default Catch2 main function to the executable.
    target_include_directories(anamoly_detection_unitTests PUBLIC
            include
            ${OpenCV_INCLUDE_DIRS}
    )
    ament_target_dependencies(anamoly_detection_unitTests
            rclcpp std_srvs std_msgs sensor_msgs tf2 tf2_ros geometry_msgs OpenCV
    )
    target_link_libraries(anamoly_detection_unitTests
            catch_ros2::catch_ros2_with_main
            BeamsDetect cracksDetect objectDetect
            ${OpenCV_LIBS}
    )

    add_test(NAME anomalyDetectionUnitTests COMMAND anamoly_detection_unitTests)
endif ()

###################################################
# Create the compilation database for clangd and move it out of the build dir.
###################################################
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

###################################################
# Doxygen Rules
# ref: https://cmake.org/cmake/help/latest/module/FindDoxygen.html
###################################################
if (DOXYGEN_FOUND)
    set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)
    set(DOXYGEN_COLLABORATION_GRAPH YES)
    set(DOXYGEN_EXTRACT_ALL YES)
    set(DOXYGEN_CLASS_DIAGRAMS YES)
    set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
    set(DOXYGEN_HAVE_DOT YES)
    set(DOXYGEN_CLASS_GRAPH YES)
    set(DOXYGEN_CALL_GRAPH YES)
    set(DOXYGEN_CALLER_GRAPH YES)
    set(DOXYGEN_COLLABORATION_GRAPH YES)
    set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
    set(DOXYGEN_EXTRACT_PRIVATE YES)
    set(DOXYGEN_EXTRACT_PACKAGE YES)
    set(DOXYGEN_EXTRACT_STATIC YES)
    set(DOXYGEN_EXTRACT_LOCALMETHODS YES)
    set(DOXYGEN_UML_LOOK YES)
    set(DOXYGEN_UML_LIMIT_NUM_FIELDS 50)
    set(DOXYGEN_TEMPLATE_RELATIONS YES)
    set(DOXYGEN_DOT_GRAPH_MAX_NODES 100)
    set(DOXYGEN_MAX_DOT_GRAPH_DEPTH 0)
    set(DOXYGEN_DOT_TRANSPARENT YES)
else ()
    message(FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation")
endif ()

###################################
# Specify Targets
###################################
# create a target to build documentation
doxygen_add_docs(docs           # target name
        # List of files or directories
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/test
)

#include_directories(
#        include
#)

###################################
# Misaligned Beams detection
###################################
add_library(BeamsDetect src/DetectMisAlignedBeams.cc)

# Add executable dependencies:
target_include_directories(BeamsDetect PUBLIC
        include/
        ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(BeamsDetect rclcpp sensor_msgs geometry_msgs tf2 tf2_ros )

###################################
# Crack detection
###################################
add_library(cracksDetect src/DetectCrack.cc)

# Add executable dependencies:
target_include_directories(cracksDetect PUBLIC
        include/
        ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(cracksDetect rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

###################################
# Object detection
###################################
add_library(objectDetect src/DetectHazardObject.cc)

# Add executable dependencies:
target_include_directories(objectDetect PUBLIC
        include/
        ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(objectDetect rclcpp sensor_msgs geometry_msgs tf2 tf2_ros )


###################################
# Main anomaly detection
###################################
add_library(anomalyDetection src/AnomalyDetection.cc)

# Add executable dependencies:
target_include_directories(anomalyDetection PUBLIC
        include/
        ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(anomalyDetection rclcpp sensor_msgs geometry_msgs tf2 tf2_ros  )

target_link_libraries(anomalyDetection BeamsDetect cracksDetect objectDetect ${OpenCV_LIBS})

###################################
# Installation
###################################

# Install executables:
install(TARGETS
        anomalyDetection
        DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

## Install some arbitrary executables
#install(PROGRAMS      # want to make the file executable (ie. chmod a+x)
##        <>
#        DESTINATION lib/${PROJECT_NAME})

ament_package()
