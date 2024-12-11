// Copyright 2024 SkynetScout
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file anamoly_detection_unit_test.cc
 * @brief This file contains all the unit test for anamoly detection module
 */

/// @file - Example unit tests using catch_ros2 utilities.

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include "DetectCrack.h"
#include "DetectMisAlignedBeams.h"
#include "DetectHazardObject.h"
#include "AnomalyDetection.h"

using catch_ros2::SimulateArgs;

TEST_CASE("Hazardous Object Detection", "[Object Detection]") {
  const auto args = SimulateArgs{
      "--ros-args -p enableCrackDetection:=false "
      "-p enableBeamDetection:=false -p enableObjectDetection:=true"
  }; // checking only Object Detection

// Initialize ROS with simulated arguments
  rclcpp::init(args.argc(), args.argv());

  auto mObjectDetection = std::make_unique<scout::DetectHazardObject>();

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
  cv::Mat input = cv::imread("./test_images/cracks.png");
  if (input.empty()) {
    std::cerr << "Error: Unable to load image!" << std::endl;
  }

  if (input.channels() != 3) {
    cv::cvtColor(input, input, cv::COLOR_GRAY2BGR);
  }
  auto ret = mObjectDetection->processImage(input);;
  REQUIRE(ret.message == "");
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

  auto mcrackDetection = std::make_unique<scout::DetectCrack>();

// Init test node
  auto node = rclcpp::Node::make_shared("Anamoly_Detection_unittest_node");

// Test that parameters are received as expected by the node
  node->declare_parameter<bool>("enableCrackDetection");
  node->declare_parameter<bool>("enableBeamDetection");
  node->declare_parameter<bool>("enableObjectDetection", true);


// Assertions
  CHECK(node->get_parameter("enableCrackDetection").get_parameter_value().get<bool>() == true);
  CHECK(node->get_parameter("enableBeamDetection").get_parameter_value().get<bool>() == false);
  CHECK(node->get_parameter("enableObjectDetection").get_parameter_value().get<bool>() == false);

  //Read Image and Pass to processImage
  cv::Mat input = cv::imread("./test_images/cracks.png");
  if (input.empty()) {
    std::cerr << "Error: Unable to load image!" << std::endl;
  }

  if (input.channels() != 3) {
    cv::cvtColor(input, input, cv::COLOR_GRAY2BGR);
  }
  auto ret = mcrackDetection->processImage(input);
  REQUIRE(ret.message == "Found cracks");
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

  cv::Mat input = cv::imread("./test_images/misaligned.png");
  if (input.empty()) {
    std::cerr << "Error: Unable to load image!" << std::endl;
  }

  if (input.channels() != 3) {
    cv::cvtColor(input, input, cv::COLOR_GRAY2BGR);
  }

  auto ret = mBeamsDetection->processImage(input);
  REQUIRE(ret.message == "Found misaligned beams");
// Shutdown ROS
  rclcpp::shutdown();
}