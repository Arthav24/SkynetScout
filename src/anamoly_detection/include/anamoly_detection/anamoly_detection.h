/**
 * @file anamoly_detection.h
 * @brief This header file includes declaration for crack detection, misaligned beams and Hazardous object detection.
 */
#ifndef ANAMOLY_DETECTION_H_
#define ANAMOLY_DETECTION_H_
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <future>

namespace scout {

class AnamolyDetection : public rclcpp::Node {
 public:
  AnamolyDetection();
  ~AnamolyDetection();
 private:

};

} // namespace end
#endif  // ANAMOLY_DETECTION_H_