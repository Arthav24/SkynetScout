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