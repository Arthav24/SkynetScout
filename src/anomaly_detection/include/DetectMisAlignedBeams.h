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
 * @brief This header file declares the MisalignedBeams class used for detecting misaligned beams in images.
 * @file DetectMisAlignedBeams.h
 * @date November 28, 2024
 * @version 1.5
 * @author Anirudh, Amogha Sunil
 * @copyright Copyright (c) 2024
 */

#ifndef ANOMALY_BEAMS_DETECTION_H_  // Header guard to prevent multiple inclusions
#define ANOMALY_BEAMS_DETECTION_H_

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
 * @class MisalignedBeams
 * @brief A class to detect misaligned beams in images. Inherits from the AnomalyBase class.
 */
class MisalignedBeams : public AnomalyBase {
 public:
  /**
   * @brief Constructor for the MisalignedBeams class.
   */
  MisalignedBeams();

  /**
   * @brief Destructor for the MisalignedBeams class.
   */
  ~MisalignedBeams();

  /**
   * @brief Method to process the input image and detect misaligned beams.
   * This method converts the input image to grayscale, processes it to find potential misalignment of beams,
   * and returns the status of the detection.
   * @startuml
   *     actor "User" as User
   *     entity "Image" as Image
   *     entity "Binary Image" as BinaryImage
   *     entity "Contours" as Contours
   *     entity "Bounding Boxes" as BoundingBoxes
   *     entity "Beam Groups" as BeamGroups
   *     entity "Misalignment Detection" as MisalignmentDetection
   *     entity "Status" as Status
   *
   *     User -> "processImage" : Calls processImage()
   *     "processImage" -> Image : Input image
   *     "processImage" -> "cv::cvtColor" : Convert to grayscale
   *     "cv::cvtColor" -> Image : Grayscale image
   *
   *     "processImage" -> "threshold" : Apply threshold
   *     "threshold" -> BinaryImage : Binary image created
   * 
   *     "processImage" -> "findContours" : Detect contours
   *     "findContours" -> Contours : Contours of beams
   *
   *     "processImage" -> "boundingRect" : Create bounding boxes from contours
   *     "boundingRect" -> BoundingBoxes : Bounding boxes
   *
   *     "processImage" -> "std::sort" : Sort bounding boxes by x position
   *     "std::sort" -> BoundingBoxes : Sorted bounding boxes
   *
   *     "processImage" -> "beamGroups" : Group bounding boxes by distance
   *     "beamGroups" -> BeamGroups : Beam groups formed
   *
   *     "processImage" -> "MisalignmentDetection" : Check for misalignment in each group
   *     "MisalignmentDetection" -> Status : Update status if misaligned
   *     "MisalignmentDetection" -> Status : Set status to OK if aligned
   *
   *     "processImage" -> Status : Return final status
   * @enduml
   * @param image The input image to be processed for misaligned beams.
   * @return The status of the anomaly detection, including a message and level.
   */
  skynet_interfaces::msg::AnomalyStatus processImage(cv::Mat) override;

 private:
  cv::Mat gray;  ///< Grayscale image for processing
  skynet_interfaces::msg::AnomalyStatus status;  ///< Status object to store detection results
};

} // namespace scout

#endif  // ANOMALY_BEAMS_DETECTION_H_  // End of header guard
