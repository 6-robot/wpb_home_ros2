#include "wpb_home_behaviors/grab.hpp"
// Chapter 11: detect, align, lift, approach, grip, raise and retreat.
// Adapted from wpr_simulation2/demo_cpp/11_grab_object.cpp.
#include "wpb_home_behaviors/runtime.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <wpr_simulation2/msg/object.hpp>

int run_grab(int argc, char ** argv, bool auto_start_default, const char * node_name)
{
  tutorial::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(node_name);
  const double align_x = tutorial::positive(node, "align_x", 1.0);
  const double align_y = node->declare_parameter("align_y", 0.0);
  const double gain = tutorial::positive(node, "align_gain", 0.8);
  const double max_speed = tutorial::positive(node, "max_align_speed", 0.15);
  const double approach_speed = tutorial::positive(node, "approach_speed", 0.1);
  const double reach = tutorial::positive(node, "gripper_reach", 0.65);
  const double seconds_per_meter = tutorial::positive(node, "approach_seconds_per_meter", 9.0);
  const double max_approach_time = tutorial::positive(node, "max_approach_time", 15.0);
  const double lift_wait = tutorial::positive(node, "lift_wait", 8.0);
  const double grip_wait = tutorial::positive(node, "grip_wait", 5.0);
  const double raise_wait = tutorial::positive(node, "raise_wait", 5.0);
  const double retreat_time = tutorial::positive(node, "retreat_time", 5.0);
  const double retreat_speed = tutorial::positive(node, "retreat_speed", 0.1);
  const double open_width = tutorial::positive(node, "open_width", 0.15);
  const double grip_width = tutorial::positive(node, "grip_width", 0.07);
  const double raise_height = tutorial::positive(node, "raise_height", 0.05);
  const double lift_offset = node->declare_parameter("lift_offset", 0.0);
  const double max_lift = tutorial::positive(node, "max_lift", 1.2);
  const double object_timeout = tutorial::positive(node, "object_timeout", 1.0);
  const double task_timeout = tutorial::positive(node, "detection_timeout", 30.0);
  if (!std::isfinite(align_y) || !std::isfinite(lift_offset) || grip_width > open_width) {
    RCLCPP_ERROR(node->get_logger(), "Invalid grab calibration parameters");
    rclcpp::shutdown(); return 1;
  }
  auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  auto mani_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpb_home/mani_ctrl", 10);
  auto cmd_pub = node->create_publisher<std_msgs::msg::String>("/wpb_home/behavior", 10);
  auto result_pub = node->create_publisher<std_msgs::msg::String>("/wpb_home/grab_result", 10);
  const bool auto_start = node->declare_parameter("auto_start", auto_start_default);
  enum Step {IDLE, WAIT, ALIGN, HAND_UP, FORWARD, GRAB, OBJ_UP, BACKWARD, DONE, FAILED};
  Step step = auto_start ? WAIT : IDLE;
  auto entered = tutorial::Clock::now();
  auto observed = tutorial::Clock::time_point{};
  double x = 0, y = 0, z = 0, approach_time = 0;
  bool valid = false;
  auto command = [&](const std::string & value) {
    std_msgs::msg::String msg; msg.data = value; cmd_pub->publish(msg);
  };
  auto transition = [&](Step next) {
    step = next; entered = tutorial::Clock::now();
    RCLCPP_INFO(node->get_logger(), "Grab step: %d", static_cast<int>(step));
  };
  auto fail = [&](const char * reason) {
    RCLCPP_ERROR(node->get_logger(), "%s", reason);
    transition(FAILED);
    command("stop objects");
    tutorial::stop(vel_pub);
    std_msgs::msg::String result; result.data = "grab failed"; result_pub->publish(result);
  };
  auto hand = [&](double height, double width) {
    sensor_msgs::msg::JointState msg;
    msg.name = {"lift", "gripper"}; msg.position = {height, width};
    mani_pub->publish(msg);
  };
  auto objects = node->create_subscription<wpr_simulation2::msg::Object>(
    "/wpb_home/objects_3d", 10, [&](wpr_simulation2::msg::Object::ConstSharedPtr msg) {
      if (step != WAIT && step != ALIGN) {return;}
      valid = false;
      if (msg->header.frame_id != "base_footprint" || msg->x.empty() ||
        msg->y.size() != msg->x.size() || msg->z.size() != msg->x.size()) {return;}
      if (!std::isfinite(msg->x[0]) || !std::isfinite(msg->y[0]) || !std::isfinite(msg->z[0])) {return;}
      x = msg->x[0]; y = msg->y[0]; z = msg->z[0] + lift_offset;
      valid = x > reach && x < 2.0 && std::abs(y) < 0.6 && z >= 0 && z + raise_height <= max_lift;
      if (valid) {observed = tutorial::Clock::now();}
    });
  auto behavior = node->create_subscription<std_msgs::msg::String>(
    "/wpb_home/behavior", 10, [&](std_msgs::msg::String::ConstSharedPtr msg) {
      if (msg->data == "start grab") {
        if (step == IDLE || step == DONE || step == FAILED) {
          valid = false;
          observed = tutorial::Clock::time_point{};
          transition(WAIT);
          command("start objects");
        } else {
          RCLCPP_WARN(node->get_logger(), "Grab already active; duplicate start ignored");
        }
      } else if ((msg->data == "stop grab" || msg->data == "grab stop") &&
                 step != IDLE && step != DONE && step != FAILED) {
        fail("Grab stopped by command");
      }
    });
  rclcpp::WallRate rate(30);
  auto last_request = tutorial::Clock::time_point{};
  while (tutorial::running()) {
    rclcpp::spin_some(node);
    geometry_msgs::msg::Twist velocity;
    const double elapsed = tutorial::age(entered);
    switch (step) {
      case IDLE:
        break;
      case WAIT:
        if (tutorial::age(last_request) > 0.5) {
          command("start objects"); last_request = tutorial::Clock::now();
        }
        if (valid && tutorial::age(observed) <= object_timeout && mani_pub->get_subscription_count() > 0) {
          transition(ALIGN);
        } else if (elapsed > task_timeout) {fail("No valid object or arm driver before timeout");}
        break;
      case ALIGN:
        if (elapsed > task_timeout) {fail("Object alignment timed out"); break;}
        if (!valid || tutorial::age(observed) > object_timeout) {break;}
        if (std::abs(x - align_x) > 0.02 || std::abs(y - align_y) > 0.01) {
          velocity.linear.x = std::clamp((x - align_x) * gain, -max_speed, max_speed);
          velocity.linear.y = std::clamp((y - align_y) * gain, -max_speed, max_speed);
        } else {
          approach_time = (x - reach) * seconds_per_meter;
          if (approach_time <= 0 || approach_time > max_approach_time) {
            fail("Approach duration outside calibrated range"); break;
          }
          command("stop objects"); hand(z, open_width); transition(HAND_UP);
        }
        break;
      case HAND_UP:
        if (elapsed >= lift_wait) {transition(FORWARD);}
        break;
      case FORWARD:
        if (elapsed < approach_time) {velocity.linear.x = approach_speed;}
        else {hand(z, grip_width); transition(GRAB);}
        break;
      case GRAB:
        if (elapsed >= grip_wait) {hand(z + raise_height, grip_width); transition(OBJ_UP);}
        break;
      case OBJ_UP:
        if (elapsed >= raise_wait) {transition(BACKWARD);}
        break;
      case BACKWARD:
        if (elapsed < retreat_time) {velocity.linear.x = -retreat_speed;}
        else {
          transition(DONE);
          tutorial::stop(vel_pub);
          std_msgs::msg::String result; result.data = "grab done"; result_pub->publish(result);
        }
        break;
      case DONE:
      case FAILED:
        break;
    }
    // Only the active grab owns /cmd_vel. Idle and terminal states release Nav2 control.
    if (step >= WAIT && step <= BACKWARD) { vel_pub->publish(velocity); }
    rate.sleep();
  }
  command("stop objects");
  if (step >= WAIT && step <= BACKWARD) { tutorial::stop(vel_pub); }
  rclcpp::shutdown();
  return step == FAILED ? 1 : 0;
}
