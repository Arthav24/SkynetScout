// Copyright 2024 SkynetScout
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief This source file includes definition for manager.
 * Manager node is responsible for orchestrating whole operations
 * @file SkynetManager.cc
 * @date November 19 2024
 * @version 1.5
 * @author Anirudh S
 * @copyright Copyright (c) 2024
 */

#include "skynet_manager/SkynetManager.h"

/*ScoutManager node*/
/**
 * @brief Constructor for ScoutManager
 */
scout::ScoutManager::ScoutManager() : Node("scout_manager") {

  /*contructor placeholder*/
  setup();
}

/**
 * @brief Destructor for ScoutManager
 */
scout::ScoutManager::~ScoutManager() { /*destructor placeholder*/ }

/**
 * Service callback for Return to home
 * @param req request
 * @param resp response
 */
void scout::ScoutManager::rthCallback(
    const std::shared_ptr<skynet_interfaces::srv::ReturnToHome::Request> req,
    std::shared_ptr<skynet_interfaces::srv::ReturnToHome::Response> resp) {
  RCLCPP_INFO(this->get_logger(), "Received Return to home request");
  resp->success = true;
  resp->message = "Going back to home";
}

/**
 * Service call back for Start Inspection service
 * @param req request
 * @param resp response
 */

void scout::ScoutManager::startCallback(
    const std::shared_ptr<skynet_interfaces::srv::StartInspection::Request> req,
    std::shared_ptr<skynet_interfaces::srv::StartInspection::Response> resp) {
  RCLCPP_INFO(this->get_logger(), "Received Return to home request");
  resp->success = true;
  resp->message = "Starting Inspection";
}

/**
 * @brief Sets up service servers
 */
void scout::ScoutManager::setup() {
  rthService_ = this->create_service<skynet_interfaces::srv::ReturnToHome>(
      "/rth", std::bind(&ScoutManager::rthCallback, this, std::placeholders::_1,
                        std::placeholders::_2));
  startService_ = this->create_service<skynet_interfaces::srv::StartInspection>(
      "/start_inspection",
      std::bind(&ScoutManager::startCallback, this, std::placeholders::_1,
                std::placeholders::_2));
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