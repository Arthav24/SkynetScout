/**
 * @file manager.h
 * @brief This header file includes declaration for manager.
 * Manager node is responsible for orchestrating whole operations
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

class ScoutManager : public rclcpp::Node {
 public:
  ScoutManager();
  ~ScoutManager();
 private:
};

} // namespace end
#endif  // SKYNET_MANAGER_H_