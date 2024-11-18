/**
 * @file anomaly_detection.h
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

#include "anamoly_detection/anamoly_crack_detect.h"
#include "anamoly_detection/anamoly_beams_detect.h"
#include "anamoly_detection/anamoly_object_detect.h"

namespace scout {

class AnamolyDetection : public rclcpp::Node {
 public:
  AnamolyDetection();
  ~AnamolyDetection();
 private:
  std::unique_ptr<scout::MisalignedBeams> mdetect_beams;
  std::unique_ptr<scout::ConcreteCracks> mdetect_cracks;
  std::unique_ptr<scout::HObject> mdetect_object;
};

} // namespace end
#endif  // ANAMOLY_DETECTION_H_