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
 * @file DetectCrack.cc
 * @date November 19, 2024
 * @version 1.5
 * @author Anirudh S, Amogha Sunil
 * @copyright Copyright (c) 2024
 */

#include "DetectCrack.h"  

/* Constructor for the DetectCrack class */
scout::DetectCrack::DetectCrack() {
  // Initialize the status message for crack detection
  status = skynet_interfaces::msg::AnomalyStatus();
  status.name = "Concrete Crack";  // Set the name of the anomaly
  status.message = "";             // Initial empty message
  status.level = skynet_interfaces::msg::AnomalyStatus::OK;  // Set default anomaly level to OK
}

/* Destructor for the DetectCrack class */
scout::DetectCrack::~DetectCrack() {
  // Destructor logic (empty for now)
};

/* Method to process image frames for crack detection */
skynet_interfaces::msg::AnomalyStatus
scout::DetectCrack::processImage(cv::Mat image) {
  // Convert the input image to grayscale
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  // Apply thresholding to isolate dark pixels (likely representing cracks)
  cv::Mat binary;
  cv::threshold(gray, binary, 15, 255, cv::THRESH_BINARY_INV);  // Inverse binary thresholding for dark pixels (potential cracks)

  // Apply morphological operations to clean up noise
  cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));  // Create a rectangular kernel
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, morphKernel);  // Perform morphological closing operation

  // Perform connected component analysis on the binary image
  cv::Mat labels, stats, centroids;
  int numComponents = cv::connectedComponentsWithStats(binary, labels, stats, centroids);
  bool isDetected = false;
  // Iterate over each connected component to detect cracks
  for (int i = 1; i < numComponents; i++) {  // Start from 1 to ignore the background (component 0)
  
    int area = stats.at<int>(i, cv::CC_STAT_AREA);  // Area of the connected component
    int x = stats.at<int>(i, cv::CC_STAT_LEFT);    // X-coordinate of the bounding box
    int y = stats.at<int>(i, cv::CC_STAT_TOP);     // Y-coordinate of the bounding box
    int width = stats.at<int>(i, cv::CC_STAT_WIDTH);   // Width of the bounding box
    int height = stats.at<int>(i, cv::CC_STAT_HEIGHT); // Height of the bounding box

    // Filter out small components based on area (ignoring noise or irrelevant small areas)
    if (area > 600) {  // Threshold area to only consider significant components
      cv::Rect boundingBox(x, y, width, height);  // Define the bounding box for the detected crack
      isDetected = true;
    }
  }
  // If crack is detected, update the status
  if (isDetected) {
    status.level = skynet_interfaces::msg::AnomalyStatus::CRACK;
    status.message = "Found cracks";
  } else {
    // If no misalignment is detected, set status to OK
    status.level = skynet_interfaces::msg::AnomalyStatus::OK;
    status.message = "";
  }
  return status;
}
