#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sr_node");
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "/speech/text", 10, [node](std_msgs::msg::String::ConstSharedPtr msg) {
      RCLCPP_INFO(node->get_logger(), "识别文字：%s", msg->data.c_str());
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
