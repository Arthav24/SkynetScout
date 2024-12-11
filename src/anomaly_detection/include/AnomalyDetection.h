/**
 * @file AnomalyDetection.h
 * @brief his header file includes declaration for Anomaly detection class
 * beams and Hazardous object detection.
 * @version 1.5
 * @date 2024-11-04
 * @author Anirudh S, Amogha Sunil
 * @copyright Copyright (c) 2024 
 */

#ifndef ANOMALY_DETECTION_H_
#define ANOMALY_DETECTION_H_

// Standard and ROS dependencies
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

// Custom header files for anomaly detection components
#include "AnomalyBase.h"
#include "DetectCrack.h"
#include "DetectHazardObject.h"
#include "DetectMisAlignedBeams.h"

namespace scout {

/**
 * @class AnomalyDetection
 * @brief This class handles anomaly detection for cracks, misaligned beams, and hazardous objects.
 * It subscribes to image and camera info topics, processes frames for anomalies, and publishes anomaly results.
 */
class AnomalyDetection : public rclcpp::Node {
public:
  /* Constructor */
  AnomalyDetection();

  /* Destructor */
  ~AnomalyDetection();

private:
  // Boolean flags to enable or disable each type of anomaly detection
  bool RUN_CRACK;
  bool RUN_BEAM;
  bool RUN_OBJECT;

  /**
   * @brief Main function that processes frames asynchronously to detect anomalies.
   * @return true if the processing is successful, false otherwise.
   *@startuml
   *   Start
   *   actor User
   *
   *   entity "AnomalyDetection" as AD
   *   entity "frameQueue" as FQ
   *   entity "frame_to_process" as FPT
   *   entity "cv_bridge" as CV
   *   entity "image (cv::Mat)" as Image
   *   entity "AnomalyStatus" as AS
   *   entity "anomaly_msg" as AM
   *   entity "anomaliesPublisher_" as Publisher
   *   entity "DetectCrack" as DC
   *   entity "MisalignedBeams" as MB
   *   entity "DetectHazardObject" as DHO
   * 
   *   User --> AD : Starts Node
   *
   *   AD -> FQ : Check if frameQueue has data
   *   FQ -> AD : Returns frame
   *   AD -> CV : Convert ROS image to OpenCV
   *   CV -> Image : Convert image to BGR format if needed
   *   AD -> DC : If RUN_CRACK enabled, process image asynchronously
   *   AD -> MB : If RUN_BEAM enabled, process image asynchronously
   *   AD -> DHO : If RUN_OBJECT enabled, process image asynchronously

   *   DC -> AS : Returns crack detection status
   *   MB -> AS : Returns beam detection status
   *   DHO -> AS : Returns object detection status
   *
   *   AS -> AM : If status not OK, add to anomalies
   *   AM -> Publisher : Publish anomalies
   *   AD -> AM : Clear anomalies for next iteration
   *
   *   AD -> AD : Continue processing while rclcpp::ok()
   *   AD -> Publisher : Publish empty message initially
   *
   *   FQ --> AD : Return empty if queue is empty
   *   AD -> User : Log "empty" if no data in queue
   *
   *   AD -> AD : Continue until exception or shutdown
   *   @enduml
   */
  bool run();

  /**
   * @brief Configures the node by setting up subscribers, publishers, and other necessary components.
   */
  void setup();

  /**
   * @brief Callback function for the image subscriber.
   * It processes the incoming camera image for anomalies.
   * @param frame The incoming image frame.
   */
  void ImageCallback(const sensor_msgs::msg::Image &frame);

  /**
   * @brief Callback function for the camera info subscriber.
   * It processes the incoming camera information.
   * @param info The incoming camera information.
   */
  void CaminfoCallback(const sensor_msgs::msg::CameraInfo &info);

  // Subscribers for image and camera info topics
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoSubscriber_;

  // Publisher for anomaly results
  rclcpp::Publisher<skynet_interfaces::msg::Anomalies>::SharedPtr anomaliesPublisher_;

  // Future object to handle asynchronous anomaly detection
  std::future<bool> runFuture;

  // Queue to store frames for processing
  std::queue<sensor_msgs::msg::Image> frameQueue;

  // Variable to store camera information
  sensor_msgs::msg::CameraInfo camInfo;

  // Class references for anomaly detection modules
  std::shared_ptr<scout::MisalignedBeams> mDetectBeams;  ///< Reference to misaligned beam detection module
  std::shared_ptr<scout::DetectCrack> mDetectCracks;     ///< Reference to crack detection module
  std::shared_ptr<scout::DetectHazardObject> mDetectObject;  ///< Reference to hazardous object detection module
};

} // namespace scout

#endif // ANOMALY_DETECTION_H_
