/**
 * @brief This source file includes definition for manager.
 * Manager node is responsible for orchestrating whole operations
 * @file manager.cc
 * @date November 19 2024
 * @version 1.5
 * @author Anirudh S
 * @copyright Copyright (c) 2024
 */

#include "skynet_manager/SkynetManager.h"

/*ScoutManager node*/
scout::ScoutManager::ScoutManager() : Node("scout_manager") {
  /*contructor placeholder*/
}

scout::ScoutManager::~ScoutManager() {
  /*destructor placeholder*/

}

/*main method*/
int main(int argc, char *argv[]) {
  /*intitialize roscpp*/
  rclcpp::init(argc, argv);

  /*ros spin call to run ScoutManager node*/
  rclcpp::spin(std::make_shared<scout::ScoutManager>());

  /*call shutdown*/
  rclcpp::shutdown();
  return 0;
}