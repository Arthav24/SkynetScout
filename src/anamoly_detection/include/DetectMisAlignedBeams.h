/**
 * @brief This header file includes declaration for 
          misaligned beams detection.
 * @file DetectMisAlignedBeams.h
 * @date November 19 2024
 * @version 1.5
 * @author Anirudh
 * @copyright Copyright (c) 2024
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

/**
 * @class MisalignedBeams
 * @brief Misaligned beams class
 */
class MisalignedBeams {
 public:
  /*constructor*/
  MisalignedBeams();

  /*destructor*/
  ~MisalignedBeams();
  void processImage(cv::Mat);
 private:
  /*image subscriber member*/
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSubs;

  /*camera info subscriber member */
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr mCamInfoSubs;

};

} // namespace end
#endif  // ANAMOLY_BEAMS_DETECTION_H_