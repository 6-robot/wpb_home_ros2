#include "wpb_home_behaviors/grab.hpp"
#include "wpb_home_behaviors/runtime.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wpr_simulation2/msg/object.hpp>

namespace
{
enum GrabStep
{
    STEP_IDLE,
    STEP_WAIT,
    STEP_ALIGN_OBJ,
    STEP_HAND_UP,
    STEP_FORWARD,
    STEP_GRAB,
    STEP_OBJ_UP,
    STEP_BACKWARD,
    STEP_DONE,
    STEP_FAILED
};
}

int run_grab(int argc, char ** argv, bool auto_start_default, const char * node_name)
{
    tutorial::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(node_name);

    const double align_x = tutorial::positive(node, "align_x", 1.0);
    const double align_y = node->declare_parameter("align_y", 0.0);
    const double align_gain = tutorial::positive(node, "align_gain", 0.8);
    const double max_align_speed = tutorial::positive(node, "max_align_speed", 0.15);
    const double approach_speed = tutorial::positive(node, "approach_speed", 0.1);
    const double gripper_reach = tutorial::positive(node, "gripper_reach", 0.65);
    const double approach_seconds_per_meter = tutorial::positive(node, "approach_seconds_per_meter", 9.0);
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
    const double detection_timeout = tutorial::positive(node, "detection_timeout", 30.0);
    if(!std::isfinite(align_y) || !std::isfinite(lift_offset) || grip_width > open_width)
    {
        RCLCPP_ERROR(node->get_logger(), "Invalid grab calibration parameters");
        rclcpp::shutdown();
        return 1;
    }

    auto cmd_pub = node->create_publisher<std_msgs::msg::String>(
        "/wpb_home/behavior",
        10
    );
    auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10
    );
    auto mani_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        "/wpb_home/mani_ctrl",
        10
    );
    auto result_pub = node->create_publisher<std_msgs::msg::String>(
        "/wpb_home/grab_result",
        10
    );

    const bool auto_start = node->declare_parameter("auto_start", auto_start_default);
    GrabStep grab_step = auto_start ? STEP_WAIT : STEP_IDLE;
    double object_x = 0.0;
    double object_y = 0.0;
    double object_z = 0.0;
    double forward_duration = 0.0;  // 前进时间，单位为秒。
    bool object_valid = false;
    auto step_started = tutorial::Clock::now();
    auto object_observed = tutorial::Clock::time_point{};
    auto last_request = tutorial::Clock::time_point{};

    auto publish_command = [&](const std::string & value)
    {
        std_msgs::msg::String cmd_msg;
        cmd_msg.data = value;
        cmd_pub->publish(cmd_msg);
    };
    auto change_step = [&](GrabStep next_step)
    {
        grab_step = next_step;
        step_started = tutorial::Clock::now();
        RCLCPP_INFO(node->get_logger(), "Grab step: %d", static_cast<int>(grab_step));
    };
    auto fail_grab = [&](const char * reason)
    {
        RCLCPP_ERROR(node->get_logger(), "%s", reason);
        change_step(STEP_FAILED);
        publish_command("stop objects");
        tutorial::stop(vel_pub);
        std_msgs::msg::String result_msg;
        result_msg.data = "grab failed";
        result_pub->publish(result_msg);
    };
    auto publish_mani = [&](double height, double width)
    {
        sensor_msgs::msg::JointState mani_msg;
        mani_msg.name = {"lift", "gripper"};
        mani_msg.position = {height, width};
        mani_pub->publish(mani_msg);
    };

    auto ObjectCallback = [&](wpr_simulation2::msg::Object::ConstSharedPtr msg)
    {
        if(grab_step != STEP_WAIT && grab_step != STEP_ALIGN_OBJ)
        {
            return;
        }
        object_valid = false;
        if(msg->header.frame_id != "base_footprint" || msg->x.empty() ||
            msg->y.size() != msg->x.size() || msg->z.size() != msg->x.size())
        {
            return;
        }
        if(!std::isfinite(msg->x[0]) || !std::isfinite(msg->y[0]) || !std::isfinite(msg->z[0]))
        {
            return;
        }
        object_x = msg->x[0];
        object_y = msg->y[0];
        object_z = msg->z[0] + lift_offset;
        object_valid = object_x > gripper_reach && object_x < 2.0 &&
            std::abs(object_y) < 0.6 && object_z >= 0 && object_z + raise_height <= max_lift;
        if(object_valid)
        {
            object_observed = tutorial::Clock::now();
        }
    };

    auto BehaviorCallback = [&](std_msgs::msg::String::ConstSharedPtr msg)
    {
        if(msg->data == "start grab")
        {
            if(grab_step == STEP_IDLE || grab_step == STEP_DONE || grab_step == STEP_FAILED)
            {
                object_valid = false;
                object_observed = tutorial::Clock::time_point{};
                change_step(STEP_WAIT);
                publish_command("start objects");
            }
            else
            {
                RCLCPP_WARN(node->get_logger(), "Grab already active; duplicate start ignored");
            }
        }
        else if((msg->data == "stop grab" || msg->data == "grab stop") &&
            grab_step != STEP_IDLE && grab_step != STEP_DONE && grab_step != STEP_FAILED)
        {
            fail_grab("Grab stopped by command");
        }
    };

    auto object_sub = node->create_subscription<wpr_simulation2::msg::Object>(
        "/wpb_home/objects_3d",
        10,
        ObjectCallback
    );
    auto behavior_sub = node->create_subscription<std_msgs::msg::String>(
        "/wpb_home/behavior",
        10,
        BehaviorCallback
    );

    rclcpp::WallRate loop_rate(30);
    while(tutorial::running())
    {
        rclcpp::spin_some(node);
        geometry_msgs::msg::Twist vel_msg;
        const double elapsed = tutorial::age(step_started);

        // 每轮只执行一个阶段，等待期间继续接收物体数据和停止指令。
        if(grab_step == STEP_WAIT)
        {
            if(tutorial::age(last_request) > 0.5)
            {
                publish_command("start objects");
                last_request = tutorial::Clock::now();
            }
            if(object_valid && tutorial::age(object_observed) <= object_timeout &&
                mani_pub->get_subscription_count() > 0)
            {
                change_step(STEP_ALIGN_OBJ);
            }
            else if(elapsed > detection_timeout)
            {
                fail_grab("No valid object or arm driver before timeout");
            }
        }
        else if(grab_step == STEP_ALIGN_OBJ)
        {
            const double diff_x = object_x - align_x;
            const double diff_y = object_y - align_y;
            if(elapsed > detection_timeout)
            {
                fail_grab("Object alignment timed out");
            }
            else if(object_valid && tutorial::age(object_observed) <= object_timeout)
            {
                if(std::abs(diff_x) > 0.02 || std::abs(diff_y) > 0.01)
                {
                    vel_msg.linear.x = std::clamp(diff_x * align_gain, -max_align_speed, max_align_speed);
                    vel_msg.linear.y = std::clamp(diff_y * align_gain, -max_align_speed, max_align_speed);
                }
                else
                {
                    forward_duration = (object_x - gripper_reach) * approach_seconds_per_meter;
                    if(forward_duration <= 0 || forward_duration > max_approach_time)
                    {
                        fail_grab("Approach duration outside calibrated range");
                    }
                    else
                    {
                        publish_command("stop objects");
                        publish_mani(object_z, open_width);
                        change_step(STEP_HAND_UP);
                    }
                }
            }
        }
        else if(grab_step == STEP_HAND_UP)
        {
            if(elapsed >= lift_wait)
            {
                change_step(STEP_FORWARD);
            }
        }
        else if(grab_step == STEP_FORWARD)
        {
            if(elapsed < forward_duration)
            {
                vel_msg.linear.x = approach_speed;
            }
            else
            {
                publish_mani(object_z, grip_width);
                change_step(STEP_GRAB);
            }
        }
        else if(grab_step == STEP_GRAB)
        {
            if(elapsed >= grip_wait)
            {
                publish_mani(object_z + raise_height, grip_width);
                change_step(STEP_OBJ_UP);
            }
        }
        else if(grab_step == STEP_OBJ_UP)
        {
            if(elapsed >= raise_wait)
            {
                change_step(STEP_BACKWARD);
            }
        }
        else if(grab_step == STEP_BACKWARD)
        {
            if(elapsed < retreat_time)
            {
                vel_msg.linear.x = -retreat_speed;
            }
            else
            {
                change_step(STEP_DONE);
                tutorial::stop(vel_pub);
                std_msgs::msg::String result_msg;
                result_msg.data = "grab done";
                result_pub->publish(result_msg);
            }
        }

        // 仅在抓取过程中发布速度，空闲、完成和失败时释放底盘控制。
        if(grab_step >= STEP_WAIT && grab_step <= STEP_BACKWARD)
        {
            vel_pub->publish(vel_msg);
        }
        loop_rate.sleep();
    }

    publish_command("stop objects");
    if(grab_step >= STEP_WAIT && grab_step <= STEP_BACKWARD)
    {
        tutorial::stop(vel_pub);
    }
    rclcpp::shutdown();
    return grab_step == STEP_FAILED ? 1 : 0;
}
