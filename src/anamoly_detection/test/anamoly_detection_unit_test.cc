/**
 * @file anamoly_detection_unit_test.cc
 * @brief This file contains all the unit test for anamoly detection module
 */

/// @file - Example unit tests using catch_ros2 utilities.

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include "anamoly_detection/anamoly_detection.h"
#include "anamoly_detection/anamoly_object_detect.h"
#include "anamoly_detection/anamoly_crack_detect.h"
#include "anamoly_detection/anamoly_beams_detect.h"

using catch_ros2::SimulateArgs;

TEST_CASE("parameters", "[parameters]") {
// The SimulateArgs class can be used to synthesize input arguments for rclcpp::init()
//  const auto args = SimulateArgs{"--ros-args -p param1:=-1.4 -p param3:=14"};
  const auto args = SimulateArgs{""};

// Initialize ROS with simulated arguments
  rclcpp::init(args.argc(), args.argv());

// Init test node
  auto node = rclcpp::Node::make_shared("Anamoly_Detection_unittest_node");


// Assertions
// Should be set to simulated parameter and not throw
  CHECK(node->get_parameter("enableCrackDetection").get_parameter_value().get<bool>() == true);

// Should be set to default value
  CHECK(node->get_parameter("enableBeamDetection").get_parameter_value().get<bool>() == true);

// Should be set to simulated parameter, not default value
  CHECK(node->get_parameter("enableObjectDetection").get_parameter_value().get<bool>() == true);

// Should throw since it was never initialized
  CHECK_THROWS(node->declare_parameter<bool>("enable"));

// Shutdown ROS
  rclcpp::shutdown();
}

TEST_CASE("Hazardous Object Detection", "[Object Detection]") {
  const auto args = SimulateArgs{
      "--ros-args -p enableCrackDetection:=false "
      "-p enableBeamDetection:=false -p enableObjectDetection:=true"
  }; // checking only Object Detection

// Initialize ROS with simulated arguments
  rclcpp::init(args.argc(), args.argv());

  auto mObjectDetection = std::make_unique<scout::HObject>();

// Init test node
  auto node = rclcpp::Node::make_shared("Anamoly_Detection_unittest_node");

// Test that parameters are received as expected by the node
  node->declare_parameter<bool>("enableCrackDetection");
  node->declare_parameter<bool>("enableBeamDetection");
  node->declare_parameter<bool>("enableObjectDetection", true);


// Assertions
  CHECK(node->get_parameter("enableCrackDetection").get_parameter_value().get<bool>() == false);
  CHECK(node->get_parameter("enableBeamDetection").get_parameter_value().get<bool>() == false);
  CHECK(node->get_parameter("enableObjectDetection").get_parameter_value().get<bool>() == true);

  //Read Image and Pass to processImage
  cv::Mat img;
  mObjectDetection->processImage(img);

// Shutdown ROS
  rclcpp::shutdown();
}

TEST_CASE("Crack Detection", "[crack Detection]") {
  const auto args = SimulateArgs{
      "--ros-args -p enableCrackDetection:=true "
      "-p enableBeamDetection:=false -p enableObjectDetection:=false"
  }; // checking only Object Detection

// Initialize ROS with simulated arguments
  rclcpp::init(args.argc(), args.argv());

  auto mcrackDetection = std::make_unique<scout::ConcreteCracks>();

// Init test node
  auto node = rclcpp::Node::make_shared("Anamoly_Detection_unittest_node");

// Test that parameters are received as expected by the node
  node->declare_parameter<bool>("enableCrackDetection");
  node->declare_parameter<bool>("enableBeamDetection");
  node->declare_parameter<bool>("enableObjectDetection", true);


// Assertions
  CHECK(node->get_parameter("enableCrackDetection").get_parameter_value().get<bool>() == false);
  CHECK(node->get_parameter("enableBeamDetection").get_parameter_value().get<bool>() == false);
  CHECK(node->get_parameter("enableObjectDetection").get_parameter_value().get<bool>() == true);

  //Read Image and Pass to processImage
  cv::Mat img;
  mcrackDetection->processImage(img);

// Shutdown ROS
  rclcpp::shutdown();
}

TEST_CASE("Misaligned Beams Detection", "[MBeams Detection]") {
  const auto args = SimulateArgs{
      "--ros-args -p enableCrackDetection:=false "
      "-p enableBeamDetection:=true -p enableObjectDetection:=false"
  }; // checking only Object Detection

// Initialize ROS with simulated arguments
  rclcpp::init(args.argc(), args.argv());

  auto mBeamsDetection = std::make_unique<scout::MisalignedBeams>();

// Init test node
  auto node = rclcpp::Node::make_shared("Anamoly_Detection_unittest_node");

// Test that parameters are received as expected by the node
  node->declare_parameter<bool>("enableCrackDetection");
  node->declare_parameter<bool>("enableBeamDetection");
  node->declare_parameter<bool>("enableObjectDetection", true);


// Assertions
  CHECK(node->get_parameter("enableCrackDetection").get_parameter_value().get<bool>() == false);
  CHECK(node->get_parameter("enableBeamDetection").get_parameter_value().get<bool>() == true);
  CHECK(node->get_parameter("enableObjectDetection").get_parameter_value().get<bool>() == false);

  //Read Image and Pass to processImage
  cv::Mat img;
  mBeamsDetection->processImage(img);

// Shutdown ROS
  rclcpp::shutdown();
}