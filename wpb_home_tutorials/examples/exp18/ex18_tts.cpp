#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("speak_node");
  const std::string text = "你好，欢迎使用六部工坊启智机器人";
  auto pub = node->create_publisher<std_msgs::msg::String>("/tts/text", 10);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (rclcpp::ok() && pub->get_subscription_count() == 0 &&
    std::chrono::steady_clock::now() < deadline) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  if (pub->get_subscription_count() == 0) {
    RCLCPP_ERROR(node->get_logger(), "TTS node is not listening on /tts/text");
    rclcpp::shutdown();
    return 1;
  }
  std_msgs::msg::String message;
  message.data = text;
  pub->publish(message);
  pub->wait_for_all_acked(std::chrono::seconds(2));
  RCLCPP_INFO(node->get_logger(), "已发送朗读文字：%s", text.c_str());
  rclcpp::shutdown();
  return 0;
}
