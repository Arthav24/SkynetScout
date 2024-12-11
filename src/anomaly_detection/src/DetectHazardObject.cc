/**
 * @brief This source file includes the definition for hazardous object detection.
 * It implements the `DetectHazardObject` class methods for detecting hazardous objects
 * in the provided image using various image processing techniques.
 * 
 * @file DetectHazardObject.cc
 * @date November 20, 2024
 * @version 1.5
 * @author Anirudh
 * @copyright Copyright (c) 2024
 */

#include "DetectHazardObject.h"

/* Constructor for the DetectHazardObject class */
scout::DetectHazardObject::DetectHazardObject() {
  status = skynet_interfaces::msg::AnomalyStatus();
  status.name = "Hazardous Object";  ///< Set anomaly name as "Hazardous Object"
  status.message = "";  ///< Set default message to empty
  status.level = skynet_interfaces::msg::AnomalyStatus::OK;  ///< Set default status level as OK
}

/* Destructor for the DetectHazardObject class */
scout::DetectHazardObject::~DetectHazardObject() {
  /* Destructor does nothing in this implementation */
};

/* Process Image method for the DetectHazardObject class */
skynet_interfaces::msg::AnomalyStatus
scout::DetectHazardObject::processImage(cv::Mat image) {
    // Convert the image to grayscale for further processing
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce noise in the grayscale image
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

    // Perform Canny edge detection to highlight edges in the image
    cv::Mat edges;
    cv::Canny(blurred, edges, 50, 150);

    // Use the Hough Line Transform to detect lines in the edge-detected image
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);

    // Create a clone of the original image for visualization
    cv::Mat output = image.clone();

    // Create a mask to store lines detected from the Hough Line Transform
    cv::Mat lineMask = cv::Mat::zeros(edges.size(), CV_8U);

    // Draw the detected lines onto the line mask
    for (const auto& line : lines) {
        cv::line(lineMask, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255), 2);
    }

    // Detect contours in the line mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(lineMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Filter and draw bounding boxes around continuous bending lines
    for (const auto& contour : contours) {
        // Calculate the arc length and area of the contour
        double arcLength = cv::arcLength(contour, true);
        double area = cv::contourArea(contour);

        // Consider only the continuous and bending lines based on threshold values
        if (arcLength > 250 && area > 300) {  // Thresholds for valid shapes
            cv::Rect boundingBox = cv::boundingRect(contour);
            // Draw the bounding rectangle around the detected object (currently commented out)
            // cv::rectangle(output, boundingBox, cv::Scalar(0, 0, 255), 2);  // Draw rectangle in red
        }
    }

    // Set the status message indicating a hazardous object was found
    status.message = "Found hazard";  
    return status;  ///< Return the status containing the result of the detection
}
