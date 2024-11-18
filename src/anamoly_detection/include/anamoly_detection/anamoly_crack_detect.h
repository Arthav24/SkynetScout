/**
 * @file anamoly_crack_detect.h
 * @brief This header file includes declaration for crack detection.
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

class ConcreteCracks {
 public:
  ConcreteCracks();
  ~ConcreteCracks();
 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSubs;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo >::SharedPtr mCamInfoSubs;
  void prcessImage(cv::Mat);

};

} // namespace end
#endif  // ANAMOLY_CRACK_DETECTION_H_