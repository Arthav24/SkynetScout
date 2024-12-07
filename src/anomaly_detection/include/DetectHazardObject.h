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