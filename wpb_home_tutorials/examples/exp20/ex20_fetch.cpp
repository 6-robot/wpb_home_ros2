// 先到 kitchen，抓取，再携带饮料到 guest。由独立行为节点执行抓取。
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "wpb_home_tutorials/runtime.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  tutorial::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fetch_node");
  const auto kitchen = node->declare_parameter<std::string>("pickup_waypoint", "kitchen");
  const auto guest = node->declare_parameter<std::string>("delivery_waypoint", "guest");
  const auto discovery_timeout = tutorial::positive(node, "discovery_timeout", 30.0);
  const auto navigation_timeout = tutorial::positive(node, "navigation_timeout", 180.0);
  const auto grab_timeout = tutorial::positive(node, "grab_timeout", 120.0);
  if (kitchen.empty() || guest.empty() || kitchen == guest) {
    RCLCPP_ERROR(node->get_logger(), "pickup and delivery waypoints must be distinct and nonempty");
    rclcpp::shutdown();
    return 1;
  }
  auto nav_pub = node->create_publisher<std_msgs::msg::String>("/waterplus/navi_waypoint", 10);
  auto behavior_pub = node->create_publisher<std_msgs::msg::String>("/wpb_home/behavior", 10);
  auto task_pub = node->create_publisher<std_msgs::msg::String>("/wpb_home/fetch_result", 10);
  auto nav_action = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node, "navigate_to_pose");
  enum Stage {WAIT, TO_KITCHEN, GRAB, TO_GUEST, DONE, FAILED};
  Stage stage = WAIT;
  auto entered = tutorial::Clock::now();
  auto send = [](const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub,
                 const std::string & value) {
    std_msgs::msg::String msg;
    msg.data = value;
    pub->publish(msg);
  };
  auto transition = [&](Stage next) {
    stage = next;
    entered = tutorial::Clock::now();
  };
  auto fail = [&](const std::string & reason) {
    RCLCPP_ERROR(node->get_logger(), "%s", reason.c_str());
    if (stage == TO_KITCHEN || stage == TO_GUEST) { send(nav_pub, "cancel"); }
    if (stage == GRAB) { send(behavior_pub, "stop grab"); }
    transition(FAILED);
    send(task_pub, "fetch failed: " + reason);
  };
  auto nav_result = node->create_subscription<std_msgs::msg::String>(
    "/waterplus/navi_result", 10, [&](std_msgs::msg::String::ConstSharedPtr msg) {
      if (stage != TO_KITCHEN && stage != TO_GUEST) { return; }
      if (msg->data != "navi done") { fail(msg->data); return; }
      if (stage == TO_KITCHEN) {
        transition(GRAB);
        send(behavior_pub, "start grab");
        RCLCPP_INFO(node->get_logger(), "已到取物点，启动抓取");
      } else {
        transition(DONE);
        send(task_pub, "fetch done");
        RCLCPP_INFO(node->get_logger(), "已到递送点。请人工确认饮料仍被夹持并取下。");
      }
    });
  auto grab_result = node->create_subscription<std_msgs::msg::String>(
    "/wpb_home/grab_result", 10, [&](std_msgs::msg::String::ConstSharedPtr msg) {
      if (stage != GRAB) { return; }
      if (msg->data != "grab done") { fail(msg->data); return; }
      transition(TO_GUEST);
      send(nav_pub, guest);
      RCLCPP_INFO(node->get_logger(), "抓取流程完成，导航到递送点");
    });
  rclcpp::WallRate rate(20);
  while (tutorial::running() && stage != DONE && stage != FAILED) {
    rclcpp::spin_some(node);
    if (stage == WAIT) {
      if (nav_pub->get_subscription_count() > 0 &&
          behavior_pub->get_subscription_count() > 0 &&
          nav_result->get_publisher_count() > 0 &&
          grab_result->get_publisher_count() > 0 &&
          node->get_service_names_and_types().count("/waterplus/get_waypoint_name") > 0 &&
          nav_action->action_server_is_ready()) {
        transition(TO_KITCHEN);
        send(nav_pub, kitchen);
        RCLCPP_INFO(node->get_logger(), "开始前往取物点 %s", kitchen.c_str());
      } else if (tutorial::age(entered) > discovery_timeout) {
        fail("navigation or grab behavior is unavailable");
      }
    } else if ((stage == TO_KITCHEN || stage == TO_GUEST) &&
               tutorial::age(entered) > navigation_timeout) {
      fail("navigation timed out");
    } else if (stage == GRAB && tutorial::age(entered) > grab_timeout) {
      fail("grab timed out");
    }
    rate.sleep();
  }
  if (stage == TO_KITCHEN || stage == TO_GUEST) { send(nav_pub, "cancel"); }
  if (stage == GRAB) { send(behavior_pub, "stop grab"); }
  if (!tutorial::running()) {
    send(task_pub, "fetch canceled");
    for (int i = 0; i < 10; ++i) { rclcpp::spin_some(node); rclcpp::sleep_for(100ms); }
  }
  const bool success = stage == DONE;
  rclcpp::shutdown();
  return success ? 0 : 1;
}
