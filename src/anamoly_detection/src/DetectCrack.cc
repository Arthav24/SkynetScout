/**
 * @brief This header file includes declaration for crack detection.
 * @file DetectCrack.cc
 * @date November 19 2024
 * @version 1.5
 * @author Anirudh S
 * @copyright Copyright (c) 2024
 */

#include "DetectCrack.h"

/*Detect crack class constructor*/
scout::DetectCrack::DetectCrack() {
  mCamInfoSubs = nullptr;
  mImageSubs = nullptr;
}
scout::DetectCrack::~DetectCrack() {
 /*destructor*/
};

/*process image frames method for detection*/
void scout::DetectCrack::processImage(cv::Mat) {

}