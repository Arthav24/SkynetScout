/**
 * @brief This header file includes declaration for manager.
 * Manager node is responsible for orchestrating whole operations
 * @file manager.h
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