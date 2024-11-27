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
 * @brief This header file includes declaration for manager.
 * Manager node is responsible for orchestrating whole operations
 * @file SkynetManager.h
 * @date November 19 2024
 * @version 1.5
 * @author Anirudh
 * @copyright Copyright (c) 2024
 */

#ifndef SKYNET_MANAGER_H_
#define SKYNET_MANAGER_H_
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <future>

namespace scout {

/**
 * @class ScoutManager
 * @brief Manager Class managing the entire pipeline
 */
class ScoutManager : public rclcpp::Node {
 public:
  /*constructor*/
  ScoutManager();

  /*destructor*/
  ~ScoutManager();
 private:
};

} // namespace 
#endif  // SKYNET_MANAGER_H_