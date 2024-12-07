/**
 * BSD 3-Clause License
 * @file AnomalyDetection.h
 * @brief This header file includes declaration for crack detection, misaligned
 * beams and Hazardous object detection.
 * @version 1.0
 * @date 2024-11-04
 * @author Anirudh S, Amogha Sunil
 */

#ifndef ANOMALY_DETECTION_H_
#define ANOMALY_DETECTION_H_

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <skynet_interfaces/msg/anomalies.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include "AnomalyBase.h"
#include "DetectCrack.h"
#include "DetectHazardObject.h"
#include "DetectMisAlignedBeams.h"

namespace scout {

/**
 * @class AnomalyDetection
 * @brief AnomalyDetection base class
 */
class AnomalyDetection : public rclcpp::Node {
public:
  /*constructor*/
  AnomalyDetection();

  /*destructor*/
  ~AnomalyDetection();

private:
  /**
   * @brief This function acts as main runnable in library
   */
  bool run();

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();
  /**
   * @brief Camera image subscriber
   * @param frame
   */
  void ImageCallback(const sensor_msgs::msg::Image &frame);
  /**
   * @brief Camera info subscriber
   * @param info
   */
  void CaminfoCallback(const sensor_msgs::msg::CameraInfo &info);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoSubscriber_;
  rclcpp::Publisher<skynet_interfaces::msg::Anomalies>::SharedPtr anomaliesPublisher_;


  std::future<bool> runFuture;
  std::queue<sensor_msgs::msg::Image> frameQueue;
  sensor_msgs::msg::CameraInfo camInfo;

  /*class reference to MisalignedBeams */
    std::shared_ptr<scout::MisalignedBeams> mDetectBeams;

  /*class reference to ConcreteCracks */
    std::shared_ptr<scout::DetectCrack> mDetectCracks;

  /*class reference to Hazardous Object detection */
    std::shared_ptr<scout::DetectHazardObject> mDetectObject;
};

} // namespace scout
#endif // ANOMALY_DETECTION_H_