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
 * @brief This source file includes definition Anomaly detection
 * @file AnomalyDetection.cc
 * @date November 20 2024
 * @version 1.5
 * @author Anirudh S, Amogha Sunil
 * @copyright Copyright (c) 2024
 */

#include "AnomalyDetection.h"

/* Anomaly detection node */
scout::AnomalyDetection::AnomalyDetection() : Node("anamoly_detector") {

  // Declare parameters for enabling different types of anomaly detection
  this->declare_parameter<bool>("enableCrackDetection", true);
  this->declare_parameter<bool>("enableBeamDetection", true);
  this->declare_parameter<bool>("enableObjectDetection", true);

  // Retrieve parameter values for each type of detection
  RUN_CRACK = this->get_parameter("enableCrackDetection")
                  .get_parameter_value()
                  .get<bool>();
  RUN_BEAM = this->get_parameter("enableBeamDetection")
                 .get_parameter_value()
                 .get<bool>();
  RUN_OBJECT = this->get_parameter("enableObjectDetection")
                   .get_parameter_value()
                   .get<bool>();
  
  // Setup required components for anomaly detection
  setup();

  // Create instances of detection modules
  mDetectObject = std::make_shared<scout::DetectHazardObject>();
  mDetectBeams = std::make_shared<scout::MisalignedBeams>();
  mDetectCracks = std::make_shared<scout::DetectCrack>();

  // Start the anomaly detection process asynchronously
  runFuture = std::async(std::launch::async, &AnomalyDetection::run, this);
}

scout::AnomalyDetection::~AnomalyDetection() {
  /* Destructor */
};

bool scout::AnomalyDetection::run() {
  // Main loop to process images and detect anomalies
  try {

    // Future objects to handle anomaly detection results asynchronously
    std::future<skynet_interfaces::msg::AnomalyStatus> crackDetectFut;
    std::future<skynet_interfaces::msg::AnomalyStatus> beamDetectFut;
    std::future<skynet_interfaces::msg::AnomalyStatus> objectDetectFut;

    // Image frame to process
    sensor_msgs::msg::Image frame_to_process;

    // Set the rate of the loop (60 Hz)
    rclcpp::Rate rate(60);

    // Create message object to hold anomalies and publish them
    auto anomaly_msg = skynet_interfaces::msg::Anomalies();
    anomaly_msg.header.stamp = this->now();

    // Publish initial empty message to the topic
    anomaliesPublisher_->publish(anomaly_msg);
    anomaly_msg.anomalies.clear();

    // Main loop
    while (rclcpp::ok()) {

      // Check if there are images in the frame queue
      if (frameQueue.size() > 0) {
        frame_to_process = frameQueue.front(); // Get the front frame

        // Convert ROS image message to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
            frame_to_process, sensor_msgs::image_encodings::BGR8);

        // Access the cv::Mat object (image)
        cv::Mat image = cv_ptr->image;

        // Ensure the image has 3 channels (BGR)
        if (image.channels() != 3) {
          cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        }

        // Run Crack detection if enabled
        if (RUN_CRACK) {
          crackDetectFut =
              std::async(std::launch::async, &DetectCrack::processImage,
                         mDetectCracks, image);
        }

        // Run Beam detection if enabled
        if (RUN_BEAM) {
          beamDetectFut =
              std::async(std::launch::async, &MisalignedBeams::processImage,
                         mDetectBeams, image);
        }

        // Run Object detection if enabled
        if (RUN_OBJECT) {
          objectDetectFut =
              std::async(std::launch::async, &DetectHazardObject::processImage,
                         mDetectObject, image);
        }

        // Get the results for beam detection
        auto beam_ = beamDetectFut.get();
        if (beam_.level != skynet_interfaces::msg::AnomalyStatus::OK) {
          anomaly_msg.anomalies.push_back(beam_);
        }

        // Get the results for crack detection
        auto crack_ = crackDetectFut.get();
        if (crack_.level != skynet_interfaces::msg::AnomalyStatus::OK) {
          anomaly_msg.anomalies.push_back(crack_);
        }

        // Get the results for object detection
        auto obj_ = objectDetectFut.get();
        if (obj_.level != skynet_interfaces::msg::AnomalyStatus::OK) {
          anomaly_msg.anomalies.push_back(obj_);
        }

        // Update the timestamp and publish anomalies
        anomaly_msg.header.stamp = this->now();
        anomaliesPublisher_->publish(anomaly_msg);

        // Clear anomalies for the next iteration
        anomaly_msg.anomalies.clear();

      } else {
        RCLCPP_INFO(this->get_logger(), "empty"); // Log if frame queue is empty
      }

      // Sleep to maintain the loop rate
      rate.sleep();
    }
    return true;
  } catch (const std::exception &e) {
    return false; // Return false if an exception occurs
  }
}

/**
 * @brief Sets up subscribers, publishers, etc. to configure the node
 */
void scout::AnomalyDetection::setup() {

  // Create subscription to receive image data
  imgSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&AnomalyDetection::ImageCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'",
              imgSubscriber_->get_topic_name());

  // Create subscription to receive camera information
  camInfoSubscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 10,
      std::bind(&AnomalyDetection::CaminfoCallback, this,
                std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'",
              camInfoSubscriber_->get_topic_name());

  // Create publisher to send anomaly messages
  anomaliesPublisher_ =
      this->create_publisher<skynet_interfaces::msg::Anomalies>("/anomalies",
                                                                10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'",
              anomaliesPublisher_->get_topic_name());
}

void scout::AnomalyDetection::ImageCallback(
    const sensor_msgs::msg::Image &frame) {
  // Check if the frame queue is full and clear it if necessary
  if (frameQueue.size() > 100) {
    std::queue<sensor_msgs::msg::Image> empty;
    std::swap(frameQueue, empty);
    RCLCPP_WARN(this->get_logger(), "Queue Size > 100, cleared");
  } else {
    frameQueue.push(frame); // Add the new frame to the queue
  }
}

void scout::AnomalyDetection::CaminfoCallback(
    const sensor_msgs::msg::CameraInfo &info) {
  // Store the camera information (validation can be added later)
  camInfo = info;
  
  // Stop the camera info subscriber after receiving the first message
  camInfoSubscriber_.reset();
}
