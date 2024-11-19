/**
 * @file anamoly_beams_detect.h
 * @brief This header file includes declaration for misaligned beams detection.
 */
#ifndef ANAMOLY_BEAMS_DETECTION_H_
#define ANAMOLY_BEAMS_DETECTION_H_
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

class MisalignedBeams {
 public:
  MisalignedBeams();
  ~MisalignedBeams();
  void processImage(cv::Mat);
 private:

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSubs;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr mCamInfoSubs;

};

} // namespace end
#endif  // ANAMOLY_BEAMS_DETECTION_H_