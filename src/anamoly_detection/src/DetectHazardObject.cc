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

/*DetectHazardObject constructor*/
scout::DetectHazardObject::DetectHazardObject() {
  mCamInfoSubs = nullptr;
  mImageSubs = nullptr;

}
scout::DetectHazardObject::~DetectHazardObject() {
 /*destructor*/
};


/*frame capture process method*/
void scout::DetectHazardObject::processImage(cv::Mat) {

}