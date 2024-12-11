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
 * @brief This source file implements the MisalignedBeams class used for detecting misaligned beams in images.
 * @file DetectMisAlignedBeams.cc
 * @date November 19, 2024
 * @version 1.5
 * @author Anirudh
 * @copyright Copyright (c) 2024
 */

#include "DetectMisAlignedBeams.h"

/* Constructor for MisalignedBeams class */
scout::MisalignedBeams::MisalignedBeams() {
  // Initialize the anomaly status message
  status = skynet_interfaces::msg::AnomalyStatus();
  status.name = "Beam Misalignment";
  status.message = "";
  status.level = skynet_interfaces::msg::AnomalyStatus::OK;
}

/* Destructor for MisalignedBeams class */
scout::MisalignedBeams::~MisalignedBeams() {};

/**
 * @brief Processes an image to detect misaligned beams based on vertical pixel continuity.
 * 
 * This method converts the image to grayscale, thresholds it to create a binary image,
 * detects contours of the beams, and checks for misalignment by analyzing the vertical
 * continuity of the beams.
 * 
 * @param image The input image containing potential misaligned beams.
 * @return AnomalyStatus A message indicating the status of beam alignment detection.
 */
skynet_interfaces::msg::AnomalyStatus
scout::MisalignedBeams::processImage(cv::Mat image) {
  // Convert the image to grayscale for further processing
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  // Threshold the image to create a binary image for better beam detection
  cv::Mat binary;
  threshold(gray, binary, 100, 255, cv::THRESH_BINARY);

  // Find contours of the beams in the binary image
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Rect> boundingBoxes;

  // Loop through the contours and filter out small ones (noise)
  for (const auto &contour : contours) {
    cv::Rect box = boundingRect(contour);
    if (box.width > 50 && box.height > 20) {  // Adjusted for horizontal beams
      boundingBoxes.push_back(box);
      rectangle(image, box, cv::Scalar(0, 255, 0), 2);  // Draw bounding box on the image
    }
  }

  // Sort bounding boxes by their horizontal (x) position
  std::sort(boundingBoxes.begin(), boundingBoxes.end(),
            [](const cv::Rect &a, const cv::Rect &b) { return a.x < b.x; });

  // Group beams based on their horizontal distance
  std::vector<std::vector<cv::Rect>> beamGroups;
  std::vector<cv::Rect> currentGroup;
  const int horizontalThreshold = 50;  // Max horizontal distance to group beams

  for (size_t i = 0; i < boundingBoxes.size(); ++i) {
    if (currentGroup.empty() || abs(boundingBoxes[i].x - currentGroup.back().x) < horizontalThreshold) {
      currentGroup.push_back(boundingBoxes[i]);
    } else {
      beamGroups.push_back(currentGroup);
      currentGroup.clear();
      currentGroup.push_back(boundingBoxes[i]);
    }
  }

  if (!currentGroup.empty()) {
    beamGroups.push_back(currentGroup);
  }

  // Analyze vertical pixel continuity within each beam group to detect misalignment
  const int misalignmentThreshold = 100;  // Allowed variation in white pixel width

  for (size_t groupIndex = 0; groupIndex < beamGroups.size(); ++groupIndex) {
    const auto &group = beamGroups[groupIndex];
    bool isMisaligned = false;

    // Check each beam in the group for misalignment
    for (const auto &box : group) {
      cv::Mat roi = binary(box);  // Region of interest (ROI) for the current beam
      int bottomRow = roi.rows - 1;
      int whitePixelWidth = countNonZero(roi.row(bottomRow));  // Calculate white pixel width at the bottom

      // Check vertical pixel continuity for misalignment
      for (int row = bottomRow - 1; row >= 0; --row) {
        int currentWidth = countNonZero(roi.row(row));
        if (abs(currentWidth - whitePixelWidth) > misalignmentThreshold) {
          isMisaligned = true;
          break;
        }
      }

      // If misalignment is detected, update the status
      if (isMisaligned) {
        status.level = skynet_interfaces::msg::AnomalyStatus::MBEAMS;
        status.message = "Found misaligned beams";
        status.position.position.x = box.x;
        status.position.position.y = box.y - 10;  // Position of misaligned beam
      } else {
        // If no misalignment is detected, set status to OK
        status.level = skynet_interfaces::msg::AnomalyStatus::OK;
        status.message = "";
        status.position.position.x = 0;
        status.position.position.y = 0;
      }
    }
  }

  // Return the status after processing the image
  return status;
}
