/**
 * @brief This header file includes declaration for crack detection.
 * @file DetectCrack.h
 * @date November 19 2024
 * @version 1.5
 * @author Anirudh S
 * @copyright Copyright (c) 2024
 */

#ifndef ANAMOLY_CRACK_DETECTION_H_
#define ANAMOLY_CRACK_DETECTION_H_

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
 * @class DetectCrack
 * @brief DetectCrack class declaration
 */
class DetectCrack {
 public:
  DetectCrack();
  ~DetectCrack();
  void processImage(cv::Mat);

 private:
  /*image subscriber member*/
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSubs;

  /*camera info subscriber member */
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr mCamInfoSubs;

};

} // namespace end
#endif  // ANAMOLY_CRACK_DETECTION_H_