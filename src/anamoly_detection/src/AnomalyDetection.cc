/**
 * @brief This source file includes definition Anomaly detection
 * @file AnomalyDetection.cc
 * @date November 20 2024
 * @version 1.5
 * @author Anirudh S
 * @copyright Copyright (c) 2024
 */

#include "AnomalyDetection.h"

/*anamoly_detector node*/
scout::AnamolyDetection::AnamolyDetection(): Node("anamoly_detector") {
  mDetectBeams = std::make_unique<scout::MisalignedBeams>();
  mDetectCracks = std::make_unique<scout::DetectCrack>();
  mDetectPipeline = std::make_unique<scout::DetectHazardObject>();

}
scout::AnamolyDetection::~AnamolyDetection() {
  /*destructor*/
};


