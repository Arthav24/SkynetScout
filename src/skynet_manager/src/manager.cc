#include "skynet_manager/manager.h"

scout::ScoutManager::ScoutManager() : Node("scout_manager") {

}

scout::ScoutManager::~ScoutManager() {

}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scout::ScoutManager>());
  rclcpp::shutdown();
  return 0;
}