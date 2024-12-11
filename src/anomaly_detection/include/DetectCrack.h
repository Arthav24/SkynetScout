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
 * @brief This header file includes the declaration for crack detection.
 * It defines the `DetectCrack` class which inherits from the `AnomalyBase` class.
 * @file DetectCrack.h
 * @date November 19, 2024
 * @version 1.5
 * @author Anirudh S, Amogha Sunil
 * @copyright Copyright (c) 2024
 */

#ifndef ANOMALY_CRACK_DETECTION_H_  // Header guard to prevent multiple inclusions
#define ANOMALY_CRACK_DETECTION_H_

// Standard includes for timing, functions, and memory management
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <future>

// OpenCV header for image processing
#include <opencv4/opencv2/opencv.hpp>

// ROS message types
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <skynet_interfaces/msg/anomaly_status.hpp>

// Include the base class for anomaly detection
#include "AnomalyBase.h"

namespace scout {

/**
 * @class DetectCrack
 * @brief This class is used for detecting cracks in images.
 * It inherits from the AnomalyBase class, which is the base for all anomaly detection classes.
 * The class performs processing on an image to detect cracks and returns the detection status.
 */
class DetectCrack : public AnomalyBase {
 public:
  /**
   * @brief Default constructor for the DetectCrack class.
   * Initializes the status message for crack detection.
   */
  DetectCrack();

  /**
   * @brief Destructor for the DetectCrack class.
   */
  ~DetectCrack();

  /**
   * @brief Processes the provided image to detect cracks.
   * This method performs image processing such as grayscale conversion, thresholding,
   * morphological operations, and connected component analysis to detect cracks in the image.
   * 
   * @param image The input image to be processed for crack detection.
   * @startuml
   *   actor User
   *   participant "DetectCrack" as DC
   *   participant "cv::Mat" as Mat
   *   participant "skynet_interfaces::msg::AnomalyStatus" as Status
   *   participant "cv::Rect" as Rect
   *   participant "cv::cvtColor" as CvColor
   *   participant "cv::threshold" as CvThreshold
   *   participant "cv::morphologyEx" as CvMorphology
   *   participant "cv::connectedComponentsWithStats" as CvConnect
   * 
   *   User -> DC : Calls processImage(cv::Mat image)
   *   DC -> CvColor : Convert image to grayscale
   *   CvColor -> Mat : gray = cv::COLOR_BGR2GRAY
   *   DC -> CvThreshold : Apply threshold to isolate dark pixels
   *   CvThreshold -> Mat : binary
   *   DC -> CvMorphology : Apply morphological closing operation
   *   CvMorphology -> Mat : binary
   *   DC -> CvConnect : Perform connected component analysis
   *   CvConnect -> Mat : labels, stats, centroids
   *   DC -> DC : Iterate over connected components
   *   DC -> Status : Set status message ("Found cracks")
   *   DC -> User : Return status
   *   @enduml
   * @return The detection status, which includes the detection message and level.
   */
  skynet_interfaces::msg::AnomalyStatus processImage(cv::Mat) override;

 private:
   // Member variables

   /**
    * @brief The grayscale image used for processing.
    * This is used in various image processing operations such as thresholding and morphological transformations.
    */
   cv::Mat gray;

   /**
    * @brief Status object to store detection results.
    * This object holds the detection level, name, and message related to crack detection.
    */
   skynet_interfaces::msg::AnomalyStatus status;
};

} // namespace scout

#endif  // ANOMALY_CRACK_DETECTION_H_  // End of header guard
