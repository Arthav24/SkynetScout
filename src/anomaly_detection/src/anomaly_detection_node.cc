/**
 * @file anomaly_detection_node.cc
 * @brief This is the executable for the Anomaly Detection module. It initializes 
 *        the ROS 2 node and starts the processing of anomaly detection tasks.
 * @version 1.5
 * @date 2024-12-03
 * @author Anirudh S, Amogha Sunil
 * @copyright Copyright (c) 2024 
 */

#include "AnomalyDetection.h"  // Include the header for AnomalyDetection class
#include <rclcpp/rclcpp.hpp>    // Include ROS 2 rclcpp library for node and communication

int main(int argc, char * argv[])
{
  // Initialize the ROS 2 communication system
  rclcpp::init(argc, argv);

  // Create an instance of the AnomalyDetection node
  auto node = std::make_shared<scout::AnomalyDetection>();

  // Log a message indicating the initiation of the Anomaly Detection module
  RCLCPP_INFO(node->get_logger(), "Initiating module: Anomaly Detection");

  // Spin the node, keeping it active to process incoming messages and tasks
  rclcpp::spin(node);

  // Shut down the ROS 2 communication system after the node has finished
  rclcpp::shutdown();

  // Return 0 indicating the program has completed successfully
  return 0;
}
