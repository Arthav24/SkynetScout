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
      // TODO: Draw the bounding rectangle on the image (commented out for now)
      // cv::rectangle(output, boundingBox, cv::Scalar(0, 0, 255), 2);  // Draw rectangle in red color
    }
  }

  // Set the status message for crack detection
  status.message = "Found cracks";  // Message indicating cracks have been detected
  return status;  // Return the status of the anomaly detection
}
