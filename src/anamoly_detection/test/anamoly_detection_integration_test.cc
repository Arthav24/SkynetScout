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
    Node = rclcpp::Node::make_shared("Anamoly Detection Integration Node");
    Logger = Node->get_logger();
  }

  ~ADTestFixture() {
  }

 protected:
  rclcpp::Node::SharedPtr Node;
};

TEST_CASE_METHOD (ADTestFixture, "Check for 2D Lidar scan", "[integration][AnamolyDetection][Lidar]") {
// Create an executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

// Define variables for message capture and subscription
//  sensor_msgs::msg::LaserScan::SharedPtr received_message;
  bool message_received = false;

// Create a subscription to the `Talker` topic
  auto subscription = Node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      10,
      [&message_received, this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//        received_message = msg;
        message_received = true;
        RCLCPP_INFO_STREAM(Node->get_logger(), "RCVD a LaserScan");
      });
// Add the subscription node to the executor
  executor->add_node(Node);
// Run the executor for a short period to allow message processing
  auto start_time = std::chrono::steady_clock::now();
  while (!
      message_received && (std::chrono::steady_clock::now()
      - start_time) < std::chrono::seconds(2)) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)
    );
  }

// Cleanup
  executor->cancel();
  rclcpp::shutdown();

// Test assertions
  REQUIRE(message_received); // Ensure a message was received
}

TEST_CASE_METHOD (ADTestFixture, "Check for OAKD camera RGB image", "[integration][AnamolyDetection][RGB]") {
// Create an executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

// Define variables for message capture and subscription
//  sensor_msgs::msg::LaserScan::SharedPtr received_message;
  bool message_received = false;

// Create a subscription to the `Talker` topic
  auto subscription = Node->create_subscription<sensor_msgs::msg::Image>(
      "/oakd/rgb/preview/image_raw",
      10,
      [&message_received, this](const sensor_msgs::msg::Image::SharedPtr msg) {
        message_received = true;
        RCLCPP_INFO_STREAM(Node->get_logger(), "RCVD a RGB Image");
      });
// Add the subscription node to the executor
  executor->add_node(Node);
// Run the executor for a short period to allow message processing
  auto start_time = std::chrono::steady_clock::now();
  while (!
      message_received && (std::chrono::steady_clock::now()
      - start_time) < std::chrono::seconds(2)) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)
    );
  }

// Cleanup
  executor->cancel();
  rclcpp::shutdown();

// Test assertions
  REQUIRE(message_received); // Ensure a message was received
}

TEST_CASE_METHOD (ADTestFixture, "Check for OAKD camera info", "[integration][AnamolyDetection][RGBinfo]") {
// Create an executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

// Define variables for message capture and subscription
//  sensor_msgs::msg::LaserScan::SharedPtr received_message;
  bool message_received = false;

// Create a subscription to the `Talker` topic
  auto subscription = Node->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/oakd/rgb/preview/camera_info",
      10,
      [&message_received, this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        message_received = true;
        RCLCPP_INFO_STREAM(Node->get_logger(), "RCVD a Camera Info");
      });
// Add the subscription node to the executor
  executor->add_node(Node);
// Run the executor for a short period to allow message processing
  auto start_time = std::chrono::steady_clock::now();
  while (!
      message_received && (std::chrono::steady_clock::now()
      - start_time) < std::chrono::seconds(2)) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)
    );
  }

// Cleanup
  executor->cancel();
  rclcpp::shutdown();

// Test assertions
  REQUIRE(message_received); // Ensure a message was received
}

TEST_CASE_METHOD (ADTestFixture, "Check for OAKD Depth pointcloud", "[integration][AnamolyDetection][Depth]") {
// Create an executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

// Define variables for message capture and subscription
//  sensor_msgs::msg::LaserScan::SharedPtr received_message;
  bool message_received = false;

// Create a subscription to the `Talker` topic
  auto subscription = Node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/oakd/rgb/preview/depth/points",
      10,
      [&message_received, this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        message_received = true;
        RCLCPP_INFO_STREAM(Node->get_logger(), "RCVD a depth point cloud");
      });
// Add the subscription node to the executor
  executor->add_node(Node);
// Run the executor for a short period to allow message processing
  auto start_time = std::chrono::steady_clock::now();
  while (!
      message_received && (std::chrono::steady_clock::now()
      - start_time) < std::chrono::seconds(2)) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)
    );
  }

// Cleanup
  executor->cancel();
  rclcpp::shutdown();

// Test assertions
  REQUIRE(message_received); // Ensure a message was received
}

TEST_CASE_METHOD (ADTestFixture, "Check for OAKD camera info", "[integration][AnamolyDetection][tf]") {

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create an executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  geometry_msgs::msg::TransformStamped t;
  bool tf_received = false;
  try {
    t = tf_buffer_->lookupTransform(
        "oakd_rgb_camera_optical_frame", "map",
        tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(
        Node->get_logger(), "Could not transform %s to %s: %s",
        "oakd_rgb_camera_optical_frame"
        "map", ex.what());
  }

// Add the subscription node to the executor
  executor->add_node(Node);
// Run the executor for a short period to allow message processing
  auto start_time = std::chrono::steady_clock::now();
  while (!tf_received && (std::chrono::steady_clock::now()
      - start_time) < std::chrono::seconds(5)) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)
    );
  }

// Cleanup
  executor->cancel();
  rclcpp::shutdown();

// Test assertions
  REQUIRE(tf_received); // Ensure a message was received
}