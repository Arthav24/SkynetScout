/**
 * @file AnomalyBase.h
 * @brief This header file includes declaration for Anomaly detection base class
 *        which is the base for crack detection, misaligned beams and hazardous 
 *        object detection.
 * @version 1.5
 * @date 2024-12-03
 * @author Anirudh S, Amogha Sunil
 * @copyright Copyright (c) 2024 
 */

#ifndef BUILD_SRC_ANOMALY_DETECTION_INCLUDE_ANOMALYBASE_H_
#define BUILD_SRC_ANOMALY_DETECTION_INCLUDE_ANOMALYBASE_H_

// Include necessary OpenCV headers for image processing
#include <opencv4/opencv2/opencv.hpp>
// Include Skynet message types for AnomalyStatus
#include <skynet_interfaces/msg/anomaly_status.hpp>

namespace scout {

// Using the factory pattern for anomaly concrete classes
/**
 * @class AnomalyBase
 * @brief This class serves as the base class for all anomaly detection classes. 
 *        It defines a common interface for different types of anomaly detection 
 *        (such as crack detection, beam misalignment, and hazardous object detection).
 */
class AnomalyBase {
public:
  /**
   * @brief Virtual destructor for cleaning up derived class instances
   */
  virtual ~AnomalyBase() {}

  /**
   * @brief Pure virtual function to process an image and detect anomalies
   * @param image The image to process (in OpenCV format)
   * @return AnomalyStatus message with the detection results
   */
  virtual skynet_interfaces::msg::AnomalyStatus processImage(cv::Mat) = 0;
};

} // namespace scout

#endif // BUILD_SRC_ANOMALY_DETECTION_INCLUDE_ANOMALYBASE_H_
