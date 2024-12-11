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
 * @author Anirudh S
 * @copyright Copyright (c) 2024
 */

#include "AnomalyDetection.h"

/*anamoly_detector node*/
scout::AnomalyDetection::AnomalyDetection() : Node("anamoly_detector") {

  this->declare_parameter<bool>("enableCrackDetection", true);
  this->declare_parameter<bool>("enableBeamDetection", true);
  this->declare_parameter<bool>("enableObjectDetection", true);

  RUN_CRACK = this->get_parameter("enableCrackDetection")
                  .get_parameter_value()
                  .get<bool>();
  RUN_BEAM = this->get_parameter("enableBeamDetection")
                 .get_parameter_value()
                 .get<bool>();
  RUN_OBJECT = this->get_parameter("enableObjectDetection")
                   .get_parameter_value()
                   .get<bool>();
  setup();
  mDetectObject = std::make_shared<scout::DetectHazardObject>();
  mDetectBeams = std::make_shared<scout::MisalignedBeams>();
  mDetectCracks = std::make_shared<scout::DetectCrack>();
  runFuture = std::async(std::launch::async, &AnomalyDetection::run, this);
}
scout::AnomalyDetection::~AnomalyDetection() {
  /*destructor*/
};

bool scout::AnomalyDetection::run() {
  // pass image to all module asynchronously and gather results to publish
  try {

    std::future<skynet_interfaces::msg::AnomalyStatus> crackDetectFut;
    std::future<skynet_interfaces::msg::AnomalyStatus> beamDetectFut;
    std::future<skynet_interfaces::msg::AnomalyStatus> objectDetectFut;
    sensor_msgs::msg::Image frame_to_process;
    rclcpp::Rate rate(60);
    auto anomaly_msg = skynet_interfaces::msg::Anomalies();
    while (rclcpp::ok()) {

      // handle null from front
      if (frameQueue.size() > 0) {
        frame_to_process = frameQueue.front();

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
            frame_to_process, sensor_msgs::image_encodings::BGR8);

        // Access the cv::Mat object
        cv::Mat image = cv_ptr->image;
        if (image.channels() != 3) {
          cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        }
        //        TODO can be simplified with template declaration
        if (RUN_CRACK) {
          crackDetectFut =
              std::async(std::launch::async, &DetectCrack::processImage,
                         mDetectCracks, image);
        }

        if (RUN_BEAM) {
          beamDetectFut =
              std::async(std::launch::async, &MisalignedBeams::processImage,
                         mDetectBeams, image);
        }

        if (RUN_OBJECT) {
          objectDetectFut =
              std::async(std::launch::async, &DetectHazardObject::processImage,
                         mDetectObject, image);
        }

        auto beam_ = beamDetectFut.get();
        if (beam_.level != skynet_interfaces::msg::AnomalyStatus::OK) {
          anomaly_msg.anomalies.push_back(beam_);
        }
        auto crack_ = beamDetectFut.get();
        if (crack_.level != skynet_interfaces::msg::AnomalyStatus::OK) {
          anomaly_msg.anomalies.push_back(crack_);
        }
        auto obj_ = beamDetectFut.get();
        if (obj_.level != skynet_interfaces::msg::AnomalyStatus::OK) {
          anomaly_msg.anomalies.push_back(obj_);
        }

        anomaly_msg.header.stamp = this->now();
        anomaliesPublisher_->publish(anomaly_msg);
        anomaly_msg.anomalies.clear();

      } else {
        RCLCPP_INFO(this->get_logger(), "empty");
      }
      rate.sleep();
    }
    return true;
  } catch (const std::exception &e) {
    return false;
  }
}

/**
 * @brief Sets up subscribers, publishers, etc. to configure the node
 */
void scout::AnomalyDetection::setup() {

  imgSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", 10,
      std::bind(&AnomalyDetection::ImageCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'",
              imgSubscriber_->get_topic_name());
  camInfoSubscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/image/info", 10,
      std::bind(&AnomalyDetection::CaminfoCallback, this,
                std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'",
              camInfoSubscriber_->get_topic_name());
  anomaliesPublisher_ =
      this->create_publisher<skynet_interfaces::msg::Anomalies>("/anomalies",
                                                                10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'",
              anomaliesPublisher_->get_topic_name());

  //  // Service server
  //  startService_ = node_->create_service<std_srvs::srv::Trigger>(
  //      "/start", std::bind(&WalkerNode::start_callback, this,
  //                          std::placeholders::_1, std::placeholders::_2));
  //  RCLCPP_INFO(node_->get_logger(), "Service server to '%s'",
  //              startService_->get_service_name());
  //
  //  // Service server
  //  stopService_ = node_->create_service<std_srvs::srv::Trigger>(
  //      "/stop", std::bind(&WalkerNode::end_callback, this,
  //      std::placeholders::_1,
  //                         std::placeholders::_2));
  //  RCLCPP_INFO(node_->get_logger(), "Service server to '%s'",
  //              startService_->get_service_name());
}

void scout::AnomalyDetection::ImageCallback(
    const sensor_msgs::msg::Image &frame) {
  // check for is queue is full then discard the old data
  if (frameQueue.size() > 100) {
    std::queue<sensor_msgs::msg::Image> empty;
    std::swap(frameQueue, empty);
    RCLCPP_WARN(this->get_logger(), "Queue Size > 100, cleared");
  } else {
    frameQueue.push(frame);
  }
}
void scout::AnomalyDetection::CaminfoCallback(
    const sensor_msgs::msg::CameraInfo &info) {

  // validate the info ? TODO
  camInfo = info;
  // stopping subscriber
  camInfoSubscriber_.reset();
}