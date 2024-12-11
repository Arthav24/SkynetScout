/**
 * @brief This header file includes the declaration for hazardous object detection.
 * It defines the `DetectHazardObject` class, which inherits from the `AnomalyBase` class.
 * The class provides functionality to detect hazardous objects in the input image.
 * 
 * @file DetectHazardObject.h
 * @date November 25, 2024
 * @version 1.5
 * @author Anirudh, Amogha Sunil
 * @copyright Copyright (c) 2024
 */

#ifndef ANOMALY_OBJECT_DETECTION_H_
#define ANOMALY_OBJECT_DETECTION_H_

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

#include "AnomalyBase.h"

namespace scout {

/**
 * @class DetectHazardObject
 * @brief This class is used for detecting hazardous objects in images.
 * It inherits from the `AnomalyBase` class and overrides the `processImage` method
 * to perform object detection on the provided image and return the detection status.
 */
class DetectHazardObject : public AnomalyBase {
 public:
  /**
   * @brief Default constructor for the DetectHazardObject class.
   * Initializes the status message for hazardous object detection.
   */
  DetectHazardObject();

  /**
   * @brief Destructor for the DetectHazardObject class.
   */
  ~DetectHazardObject();

  /**
   * @brief Processes the input image to detect hazardous objects.
   * The method applies a series of image processing steps, including grayscale conversion,
   * Gaussian blur, edge detection, and Hough Line Transform, to identify potential hazardous
   * objects in the image. It returns a status message indicating whether any hazardous object
   * was found.
   * @param image The input image to be processed for hazardous object detection.
   * @startuml
   *   title `processImage` Method - Hazard Detection 
   *   actor User
   *   entity "scout::DetectHazardObject" as DetectHazardObject
   *   entity "Image" as Image
   *   entity "Grayscale Image" as GrayscaleImage
   *   entity "Blurred Image" as BlurredImage
   *   entity "Edges" as Edges
   *   entity "lines" as Lines
   *   entity "Line Mask" as LineMask
   *   entity "Contours" as Contours
   *   entity "AnomalyStatus" as Status
   * 
   *   User --> DetectHazardObject : Calls processImage(image)
   *   DetectHazardObject --> Image : Receives input image
   *   DetectHazardObject --> GrayscaleImage : Convert to grayscale
   *   DetectHazardObject --> BlurredImage : Apply Gaussian Blur
   *   DetectHazardObject --> Edges : Canny edge detection
   *   DetectHazardObject --> Lines : Hough Line Transform
   *   DetectHazardObject --> LineMask : Draw detected lines on mask
   *   DetectHazardObject --> Contours : Find contours in the line mask
   *   DetectHazardObject --> Status : Set status message ("Found hazard")
   *   DetectHazardObject --> Status : Return status
   *   @enduml
   * @return The detection status, which includes the detection message and level.
   */
  skynet_interfaces::msg::AnomalyStatus processImage(cv::Mat) override;

 private:

   /**
    * @brief The grayscale image used for processing.
    * This image is converted from the input image for processing, including detection steps.
    */
   cv::Mat gray;

   /**
    * @brief Status object to store the results of hazardous object detection.
    * This object holds the detection message and level for hazardous objects.
    */
   skynet_interfaces::msg::AnomalyStatus status;
};

} // namespace scout

#endif  // ANOMALY_OBJECT_DETECTION_H_  // End of header guard
