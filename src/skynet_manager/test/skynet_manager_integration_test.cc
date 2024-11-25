//
// Created by arthavnuc on 11/15/24.
//
/**
 * @file integration test node.cc
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
#include "skynet_interfaces/srv/start_inspection.hpp"
#include "skynet_interfaces/srv/return_to_home.hpp"

auto Logger = rclcpp::get_logger(""); // create an initial Logger
class SMTestFixture {
 public:
  SMTestFixture() {
    testerNode = rclcpp::Node::make_shared("Skynet - IntegrationTestNode");
    Logger = testerNode->get_logger();
    testerNode->declare_parameter<double> ("test_duration");
    TEST_DURATION = testerNode->get_parameter("test_duration").get_parameter_value().get<double>();
    RCLCPP_INFO_STREAM (Logger, "Got test_duration =" << TEST_DURATION);
  }

  ~SMTestFixture() {
  }

 protected:
  double TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

TEST_CASE_METHOD (SMTestFixture, "test service server for start Inspection", "[service]") {

  /**
   * 4.) Now, create a client for the specific service we're looking for:
   */
  auto client = testerNode->create_client<skynet_interfaces::srv::StartInspection> ("/start_inspection");
  RCLCPP_INFO_STREAM (Logger, "/start_inspection client created");

  /**
   * 5.) Finally do the actual test:
   */
  rclcpp::Time start_time    = rclcpp::Clock().now(); // reads /clock, if "use_sim_time" is true
  bool service_found         = false;
//  rclcpp::Duration duration;
  RCLCPP_INFO_STREAM (Logger, "Performing Test...");
  auto timeout = std::chrono::milliseconds ((int) (TEST_DURATION * 1000));

  if (client->wait_for_service (timeout)) { // blocking
//    duration = (rclcpp::Clock().now() - start_time);
    service_found = true;
  }

  RCLCPP_INFO_STREAM (Logger, " service_found=" << service_found);
  CHECK (service_found); // Test assertions - check that the servie was found
}

TEST_CASE_METHOD (SMTestFixture, "test service server for Return To Home", "[service]") {

  /**
   * 4.) Now, create a client for the specific service we're looking for:
   */
  auto client = testerNode->create_client<skynet_interfaces::srv::ReturnToHome> ("/rth");
  RCLCPP_INFO_STREAM (Logger, "/rth client created");

  /**
   * 5.) Finally do the actual test:
   */
  rclcpp::Time start_time    = rclcpp::Clock().now(); // reads /clock, if "use_sim_time" is true
  bool service_found         = false;
//  rclcpp::Duration duration  = 0s;
  RCLCPP_INFO_STREAM (Logger, "Performing Test...");
  auto timeout = std::chrono::milliseconds ((int) (TEST_DURATION * 1000));

  if (client->wait_for_service (timeout)) { // blocking
//    duration = (rclcpp::Clock().now() - start_time);
    service_found = true;
  }

  RCLCPP_INFO_STREAM (Logger," service_found=" << service_found);
  CHECK (service_found); // Test assertions - check that the servie was found
}