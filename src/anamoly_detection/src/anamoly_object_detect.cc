#include "anamoly_detection/anamoly_object_detect.h"


scout::HObject::HObject() {
  mCamInfoSubs = nullptr;
  mImageSubs = nullptr;

}
scout::HObject::~HObject() {

};

void scout::HObject::processImage(cv::Mat) {

}