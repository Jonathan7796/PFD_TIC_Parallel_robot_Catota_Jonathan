
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("effort_test_node");

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/effort_controllers/commands", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  std_msgs::msg::Float64MultiArray commands;

  using namespace std::chrono_literals;

  commands.data.push_back(0);
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);

  commands.data[0] = 300;
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);

  commands.data[0] = -200;
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);

  commands.data[0] = 500;
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);
  rclcpp::shutdown();

  return 0;
}

