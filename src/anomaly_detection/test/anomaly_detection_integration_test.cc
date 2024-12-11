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

//
// Created by arthavnuc on 11/15/24.
//
/**
 * @file anamoly_detection_integration_test_node.cc
 * @brief C++ ROS2 node
 * Copyright (c)
 * All rights reserved
 *
 * This file is part of the ENPM700 Assignments. Redistribution and use in
 * source and binary forms, with or without modification, are permitted
 * exclusively under the terms of the Apache-2.0 license.
 */
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include <catch_ros2/catch_ros2.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <skynet_interfaces/msg/anomalies.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

auto Logger = rclcpp::get_logger(""); // create an initial Logger

class ADTestFixture {
 public:
  ADTestFixture() {
    Node = rclcpp::Node::make_shared("Anamoly_Detection_Integration_Node");
    Logger = Node->get_logger();
    RCLCPP_WARN(Logger,"CALLED INS");
  }

  ~ADTestFixture() {
  }

 protected:
  rclcpp::Node::SharedPtr Node;
};

TEST_CASE_METHOD (ADTestFixture, "Check for Anomalies topic", "[integration][AnamolyDetection][Anomalies]") {
// Create an executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

// Define variables for message capture and subscription
//  sensor_msgs::msg::LaserScan::SharedPtr received_message;
  bool message_received = false;

// Create a subscription to the `Talker` topic
  auto subscription = Node->create_subscription<skynet_interfaces::msg::Anomalies>(
      "/anomalies",
      10,
      [&message_received, this](const skynet_interfaces::msg::Anomalies::SharedPtr msg) {
//        received_message = msg;
        message_received = true;
        RCLCPP_INFO_STREAM(Node->get_logger(), "RCVD an Anomaly");
      });
// Add the subscription node to the executor
  executor->add_node(Node);
// Run the executor for a short period to allow message processing
  auto start_time = std::chrono::steady_clock::now();
  while (!
      message_received && (std::chrono::steady_clock::now()
      - start_time) < std::chrono::seconds(10)) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)
    );
  }

// Cleanup
  executor->cancel();
// Test assertions
  REQUIRE(message_received); // Ensure a message was received
}
