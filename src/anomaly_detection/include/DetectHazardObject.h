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
 * @brief This header file includes declaration for 
 *        hazardous object detection.
 * @file DetectHazardObject.h
 * @date November 20 2024
 * @version 1.5
 * @author Anirudh
 * @copyright Copyright (c) 2024
 */

#ifndef ANOMALY_OBJECT_DETECTION_H_
#define ANOMALY_OBJECT_DETECTION_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <future>
#include <opencv4/opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <skynet_interfaces/msg/anomaly_status.hpp>
#include "AnomalyBase.h"

namespace scout {

/**
 * @class DetectionPipeline
 * @brief Anomaly Detection class that runs the detection pipeline
 */
class DetectHazardObject : public AnomalyBase {
 public:
 /*constructor*/
  DetectHazardObject();

  /*Destructor*/
  ~DetectHazardObject();
  
  /*frame capture process*/
  skynet_interfaces::msg::AnomalyStatus processImage(cv::Mat) override;

 private:
   cv::Mat gray;
   skynet_interfaces::msg::AnomalyStatus status;
};

} // namespace end
#endif  // ANOMALY_OBJECT_DETECTION_H_