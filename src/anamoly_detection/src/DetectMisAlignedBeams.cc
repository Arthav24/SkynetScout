/**
 * @brief This header file includes declaration for 
          misaligned beams detection.
 * @file DetectMisAlignedBeams.cc
 * @date November 19 2024
 * @version 1.5
 * @author Anirudh
 * @copyright Copyright (c) 2024
 */

#include "DetectMisAlignedBeams.h"

/*constructor misaligned beams*/
scout::MisalignedBeams::MisalignedBeams() {
  mCamInfoSubs = nullptr;
  mImageSubs = nullptr;

}
scout::MisalignedBeams::~MisalignedBeams() {
 /*destructor*/
};

/*process image method*/
void scout::MisalignedBeams::processImage(cv::Mat) {

}
