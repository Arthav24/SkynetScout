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
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <skynet_interfaces/srv/return_to_home.hpp>
#include <skynet_interfaces/srv/start_inspection.hpp>
#include <string>

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
  void setup();

  void rthCallback(
      const std::shared_ptr<skynet_interfaces::srv::ReturnToHome::Request> req,
      std::shared_ptr<skynet_interfaces::srv::ReturnToHome::Response> resp);

  void startCallback(
      const std::shared_ptr<skynet_interfaces::srv::StartInspection::Request>
          req,
      std::shared_ptr<skynet_interfaces::srv::StartInspection::Response> resp);

  rclcpp::Service<skynet_interfaces::srv::ReturnToHome>::SharedPtr rthService_;
  rclcpp::Service<skynet_interfaces::srv::StartInspection>::SharedPtr
      startService_;
};

} // namespace scout
#endif // SKYNET_MANAGER_H_