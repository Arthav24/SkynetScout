//
// Created by arthavnuc on 12/3/24.
//

#ifndef BUILD_SRC_ANOMALY_DETECTION_INCLUDE_ANOMALYBASE_H_
#define BUILD_SRC_ANOMALY_DETECTION_INCLUDE_ANOMALYBASE_H_

#include <opencv4/opencv2/opencv.hpp>
#include <skynet_interfaces/msg/anomaly_status.hpp>

namespace scout {
// Using factory pattern for Anomaly concrete classes
/**
 * @class Anomaly
 * @brief This class is base class for all possible anomaly detection classes.
 */
class AnomalyBase {
public:
  virtual ~AnomalyBase() {}
  virtual skynet_interfaces::msg::AnomalyStatus processImage(cv::Mat) = 0;
};
} // namespace scout

#endif // BUILD_SRC_ANOMALY_DETECTION_INCLUDE_ANOMALYBASE_H_
