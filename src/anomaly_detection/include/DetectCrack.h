/**
 * @brief This header file includes declaration for crack detection.
 * @file DetectCrack.h
 * @date November 19 2024
 * @version 1.5
 * @author Anirudh S
 * @copyright Copyright (c) 2024
 */

#ifndef ANOMALY_CRACK_DETECTION_H_
#define ANOMALY_CRACK_DETECTION_H_

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
 * @class DetectCrack
 * @brief DetectCrack class declaration
 */
class DetectCrack : public AnomalyBase {
 public:
  DetectCrack();

  ~DetectCrack();

  skynet_interfaces::msg::AnomalyStatus processImage(cv::Mat) override;

 private:
   cv::Mat gray;
   skynet_interfaces::msg::AnomalyStatus status;
};

} // namespace end
#endif  // ANOMALY_CRACK_DETECTION_H_