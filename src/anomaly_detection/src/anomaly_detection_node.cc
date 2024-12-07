/**
* @file anomaly_detection_node.cc
 * @brief This is executable for module anomaly detection
*/

#include "AnomalyDetection.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<scout::AnomalyDetection>();
  RCLCPP_INFO(node->get_logger(),"Initiating module: Anomaly Detection");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}