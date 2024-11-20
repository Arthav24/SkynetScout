/**
 * BSD 3-Clause License
 * @file AnomalyDetection.h
 * @brief This header file includes declaration for crack detection, misaligned beams and Hazardous object detection.
 * @version 1.0
 * @date 2024-11-04
 * @author Anirudh S, Amogha Sunil
 */

#ifndef ANAMOLY_DETECTION_H_
#define ANAMOLY_DETECTION_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <future>

#include "DetectCrack.h"
#include "DetectMisAlignedBeams.h"
#include "DetectHazardObject.h"

namespace scout {

/**
 * @class AnamolyDetection
 * @brief AnamolyDetection base class
 */
class AnamolyDetection : public rclcpp::Node {
 public:
  /*constructor*/
  AnamolyDetection();

  /*destructor*/
  ~AnamolyDetection();

 private:
  /*class reference to MisalignedBeams */
  std::unique_ptr<scout::MisalignedBeams> mDetectBeams;

  /*class reference to ConcreteCracks */
  std::unique_ptr<scout::DetectCrack> mDetectCracks;

  /*class reference to SkynetPipeline */
  std::unique_ptr<scout::DetectHazardObject> mDetectPipeline;
};

} // namespace end
#endif  // ANAMOLY_DETECTION_H_