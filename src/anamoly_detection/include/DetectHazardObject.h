/**
 * @brief This header file includes declaration for 
 *        hazardous object detection.
 * @file DetectHazardObject.h
 * @date November 20 2024
 * @version 1.5
 * @author Anirudh
 * @copyright Copyright (c) 2024
 */

#ifndef ANAMOLY_OBJECT_DETECTION_H_
#define ANAMOLY_OBJECT_DETECTION_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <future>
#include <opencv4/opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace scout {

/**
 * @class DetectionPipeline
 * @brief Anomlay Detection class that runs the detection pipeline
 */
class DetectHazardObject {
 public:
 /*constructor*/
  DetectHazardObject();

  /*Destructor*/
  ~DetectHazardObject();
  
  /*frame capture process*/
  void processImage(cv::Mat);

 private:
  /*image subscriber member*/
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSubs ;

  /*camera info subscriber member */
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo >::SharedPtr mCamInfoSubs;
};

} // namespace end
#endif  // ANAMOLY_OBJECT_DETECTION_H_