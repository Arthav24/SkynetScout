/**
 * @brief This header file includes declaration for crack detection.
 * @file DetectCrack.cc
 * @date November 19 2024
 * @version 1.5
 * @author Anirudh S
 * @copyright Copyright (c) 2024
 */

#include "DetectCrack.h"

/*Detect crack class constructor*/
scout::DetectCrack::DetectCrack() {
  status = skynet_interfaces::msg::AnomalyStatus();
  status.name = "Concrete Crack";
  status.message = "";
  status.level = skynet_interfaces::msg::AnomalyStatus::OK;
}
scout::DetectCrack::~DetectCrack() {
  /*destructor*/
};

/*process image frames method for detection*/
skynet_interfaces::msg::AnomalyStatus
scout::DetectCrack::processImage(cv::Mat image) {
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  // Step 2: Apply thresholding to isolate dark pixels
  cv::Mat binary;
  cv::threshold(gray, binary, 15, 255, cv::THRESH_BINARY_INV); // Inverse binary thresholding for dark pixels

  // Optional: Apply morphological operations to clean up noise
  cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, morphKernel);

  // Step 3: Perform connected component analysis
  cv::Mat labels, stats, centroids;
  int numComponents = cv::connectedComponentsWithStats(binary, labels, stats, centroids);

  // Step 4: Draw separate bounding boxes for each connected component
//  output = image.clone(); // Copy the input image for drawing
  for (int i = 1; i < numComponents; i++) { // Start from 1 to ignore the background
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    int y = stats.at<int>(i, cv::CC_STAT_TOP);
    int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

    // Filter out small components based on area if needed
    if (area > 600) {
      cv::Rect boundingBox(x, y, width, height);
      // Draw the bounding rectangle on the image
//      cv::rectangle(output, boundingBox, cv::Scalar(0, 0, 255), 2);
    }
  }
//  TODO fill status object


  return status;
}