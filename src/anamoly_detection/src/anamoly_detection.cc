#include "anamoly_detection/anamoly_detection.h"


scout::AnamolyDetection::AnamolyDetection(): Node("anamoly_detector") {

  mdetect_beams = std::make_unique<scout::MisalignedBeams>();
  mdetect_cracks = std::make_unique<scout::ConcreteCracks>();
  mdetect_object = std::make_unique<scout::HObject>();

}
scout::AnamolyDetection::~AnamolyDetection() {

};

