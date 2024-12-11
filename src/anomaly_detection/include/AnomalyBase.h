// Copyright 2024 SkynetScout
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
