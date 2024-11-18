/**
 * @file anamoly_object_detect.h
 * @brief This header file includes declaration for hazardous object detection.
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

class HObject {
 public:
  HObject();
  ~HObject();
 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSubs ;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo >::SharedPtr mCamInfoSubs;
  void prcessImage(cv::Mat);
};

} // namespace end
#endif  // ANAMOLY_OBJECT_DETECTION_H_