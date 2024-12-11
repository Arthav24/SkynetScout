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