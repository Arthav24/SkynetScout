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

/**
 * @brief This header file includes definition for
 *        hazardous object detection.
 * @file DetectHazardObject.cc
 * @date November 20 2024
 * @version 1.5
 * @author Anirudh
 * @copyright Copyright (c) 2024
 */

#include "DetectHazardObject.h"

///*DetectHazardObject constructor*/
scout::DetectHazardObject::DetectHazardObject() {
  status = skynet_interfaces::msg::AnomalyStatus();
  status.name = "Hazardous Object";
  status.message = "";
  status.level = skynet_interfaces::msg::AnomalyStatus::OK;
}
scout::DetectHazardObject::~DetectHazardObject() {
  /*destructor*/
};

/*frame capture process method*/
skynet_interfaces::msg::AnomalyStatus
scout::DetectHazardObject::processImage(cv::Mat) {

  return status;
}