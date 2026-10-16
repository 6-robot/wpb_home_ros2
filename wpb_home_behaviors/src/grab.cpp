#include "wpb_home_behaviors/grab.hpp"
#include "wpb_home_behaviors/runtime.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wpb_home_msgs/msg/object.hpp>

namespace
{
// 默认参数集中在这里；启动时也可以通过同名 ROS 参数覆盖。
struct GrabParameters
{
    double align_x = 1.0;
    double align_y = 0.0;
    double object_tolerance_x = 0.03;
    double object_tolerance_y = 0.03;
    double move_tolerance_x = 0.02;
    double move_tolerance_y = 0.01;
    double align_speed = 0.05;
    double forward_speed = 0.1;
    double reset_wait = 0.5;
    double stop_wait = 0.5;
    double reset_tolerance = 0.001;

    // 前进距离 = forward_scale * (object_x - forward_reference)。
    double forward_scale = 0.7;
    double forward_reference = 0.7;
    double lift_offset = 0.04;
    double open_width = 0.15;
    double grip_width = 0.045;
    double raise_height = 0.05;
    double lift_wait = 18.0;
    double grip_wait = 5.0;
    double raise_wait = 5.0;

    // 范围和超时保护，距离单位为米，时间单位为秒。
    double max_lift = 1.2;
    double max_object_x = 2.0;
    double max_object_y = 0.6;
    double max_forward_distance = 1.5;
    double object_timeout = 1.0;
    double detection_timeout = 30.0;
    double motion_timeout = 30.0;
    double pose_timeout = 1.0;
};

enum GrabStep
{
    STEP_IDLE,
    STEP_WAIT,
    STEP_ALIGN_OBJ,
    STEP_ALIGN_RESET,
    STEP_ALIGN_MOVE,
    STEP_ALIGN_STOP,
    STEP_HAND_UP,
    STEP_FORWARD_RESET,
    STEP_FORWARD_MOVE,
    STEP_FORWARD_STOP,
    STEP_GRAB,
    STEP_OBJ_UP,
    STEP_BACKWARD_RESET,
    STEP_BACKWARD_MOVE,
    STEP_BACKWARD_STOP,
    STEP_DONE,
    STEP_FAILED
};
}

int run_grab(int argc, char ** argv, bool auto_start_default, const char * node_name)
{
    tutorial::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(node_name);

    GrabParameters p;
    auto positive = [&](const char * name, double & value)
    {
        value = tutorial::positive(node, name, value);
    };
    positive("align_x", p.align_x);
    p.align_y = node->declare_parameter("align_y", p.align_y);
    positive("object_tolerance_x", p.object_tolerance_x);
    positive("object_tolerance_y", p.object_tolerance_y);
    positive("move_tolerance_x", p.move_tolerance_x);
    positive("move_tolerance_y", p.move_tolerance_y);
    positive("align_speed", p.align_speed);
    positive("forward_speed", p.forward_speed);
    positive("reset_wait", p.reset_wait);
    positive("stop_wait", p.stop_wait);
    positive("reset_tolerance", p.reset_tolerance);
    positive("forward_scale", p.forward_scale);
    positive("forward_reference", p.forward_reference);
    p.lift_offset = node->declare_parameter("lift_offset", p.lift_offset);
    positive("open_width", p.open_width);
    positive("grip_width", p.grip_width);
    positive("raise_height", p.raise_height);
    positive("lift_wait", p.lift_wait);
    positive("grip_wait", p.grip_wait);
    positive("raise_wait", p.raise_wait);
    positive("max_lift", p.max_lift);
    positive("max_object_x", p.max_object_x);
    positive("max_object_y", p.max_object_y);
    positive("max_forward_distance", p.max_forward_distance);
    positive("object_timeout", p.object_timeout);
    positive("detection_timeout", p.detection_timeout);
    positive("motion_timeout", p.motion_timeout);
    positive("pose_timeout", p.pose_timeout);
    if(!std::isfinite(p.align_y) || !std::isfinite(p.lift_offset) ||
       p.grip_width > p.open_width)
    {
        RCLCPP_ERROR(node->get_logger(), "Invalid grab calibration parameters");
        rclcpp::shutdown();
        return 1;
    }

    auto cmd_pub = node->create_publisher<std_msgs::msg::String>("/wpb_home/behavior", 10);
    auto ctrl_pub = node->create_publisher<std_msgs::msg::String>("/wpb_home/ctrl", 10);
    auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto mani_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpb_home/mani_ctrl", 10);
    auto result_pub = node->create_publisher<std_msgs::msg::String>("/wpb_home/grab_result", 10);

    const bool auto_start = node->declare_parameter("auto_start", auto_start_default);
    GrabStep grab_step = auto_start ? STEP_WAIT : STEP_IDLE;
    double object_x = 0.0;
    double object_y = 0.0;
    double object_z = 0.0;
    double move_x = 0.0;
    double move_y = 0.0;
    double forward_distance = 0.0;
    bool object_valid = false;
    bool detection_requested = false;
    bool pose_diff_ready = false;
    bool pose_received = false;
    geometry_msgs::msg::Pose2D pose_diff;
    rclcpp::Time detection_start;
    auto step_started = tutorial::Clock::now();
    auto object_observed = tutorial::Clock::time_point{};
    auto pose_observed = tutorial::Clock::time_point{};

    auto active = [&]()
    {
        return grab_step != STEP_IDLE && grab_step != STEP_DONE && grab_step != STEP_FAILED;
    };
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
    auto reset_pose = [&](GrabStep next_step)
    {
        pose_diff_ready = false;
        pose_received = false;
        change_step(next_step);
        std_msgs::msg::String reset_msg;
        reset_msg.data = "pose_diff reset";
        ctrl_pub->publish(reset_msg);
    };

    auto ObjectCallback = [&](wpb_home_msgs::msg::Object::ConstSharedPtr msg)
    {
        // 每轮只取重新开启检测后的一次有效测量，运动时不更新目标。
        if(grab_step != STEP_WAIT || !detection_requested || object_valid)
        {
            return;
        }
        if(msg->header.frame_id != "base_footprint" || msg->x.empty() ||
           msg->y.size() != msg->x.size() || msg->z.size() != msg->x.size() ||
           rclcpp::Time(msg->header.stamp, node->get_clock()->get_clock_type()) < detection_start)
        {
            return;
        }
        if(!std::isfinite(msg->x[0]) || !std::isfinite(msg->y[0]) || !std::isfinite(msg->z[0]))
        {
            return;
        }
        const double height = msg->z[0] + p.lift_offset;
        if(msg->x[0] <= p.forward_reference || msg->x[0] >= p.max_object_x ||
           std::abs(msg->y[0]) >= p.max_object_y || height < 0 ||
           height + p.raise_height > p.max_lift)
        {
            return;
        }
        object_x = msg->x[0];
        object_y = msg->y[0];
        object_z = msg->z[0];
        object_valid = true;
        object_observed = tutorial::Clock::now();
    };
    auto PoseDiffCallback = [&](geometry_msgs::msg::Pose2D::ConstSharedPtr msg)
    {
        if(!std::isfinite(msg->x) || !std::isfinite(msg->y) || !std::isfinite(msg->theta))
        {
            return;
        }
        pose_diff = *msg;
        pose_received = true;
        pose_observed = tutorial::Clock::now();
        if((grab_step == STEP_ALIGN_RESET || grab_step == STEP_FORWARD_RESET ||
            grab_step == STEP_BACKWARD_RESET) && tutorial::age(step_started) >= p.reset_wait &&
           std::abs(pose_diff.x) <= p.reset_tolerance &&
           std::abs(pose_diff.y) <= p.reset_tolerance &&
           std::abs(pose_diff.theta) <= p.reset_tolerance)
        {
            pose_diff_ready = true;
        }
    };
    auto BehaviorCallback = [&](std_msgs::msg::String::ConstSharedPtr msg)
    {
        if(msg->data == "start grab")
        {
            if(!active())
            {
                object_valid = false;
                detection_requested = false;
                pose_diff_ready = false;
                pose_received = false;
                change_step(STEP_WAIT);
            }
            else
            {
                RCLCPP_WARN(node->get_logger(), "Grab already active; duplicate start ignored");
            }
        }
        else if((msg->data == "stop grab" || msg->data == "grab stop") && active())
        {
            fail_grab("Grab stopped by command");
        }
    };

    auto object_sub = node->create_subscription<wpb_home_msgs::msg::Object>(
        "/wpb_home/objects_3d", 10, ObjectCallback);
    auto pose_diff_sub = node->create_subscription<geometry_msgs::msg::Pose2D>(
        "/wpb_home/pose_diff", 1, PoseDiffCallback);
    auto behavior_sub = node->create_subscription<std_msgs::msg::String>(
        "/wpb_home/behavior", 10, BehaviorCallback);

    rclcpp::WallRate loop_rate(30);
    while(tutorial::running())
    {
        rclcpp::spin_some(node);
        geometry_msgs::msg::Twist vel_msg;
        const double elapsed = tutorial::age(step_started);

        if(grab_step == STEP_WAIT)
        {
            // 本节点也订阅 behavior，需等外部订阅者连接后再请求检测。
            if(!detection_requested && cmd_pub->get_subscription_count() > 1)
            {
                detection_start = node->now();
                publish_command("start objects");
                detection_requested = true;
            }
            if(object_valid && tutorial::age(object_observed) <= p.object_timeout &&
               mani_pub->get_subscription_count() > 0)
            {
                change_step(STEP_ALIGN_OBJ);
            }
            else if(elapsed > p.detection_timeout)
            {
                fail_grab("No valid object or arm driver before timeout");
            }
        }
        else if(grab_step == STEP_ALIGN_OBJ)
        {
            publish_command("stop objects");
            const double diff_x = object_x - p.align_x;
            const double diff_y = object_y - p.align_y;
            if(std::abs(diff_x) <= p.object_tolerance_x && std::abs(diff_y) <= p.object_tolerance_y)
            {
                forward_distance = p.forward_scale * (object_x - p.forward_reference);
                if(forward_distance <= 0 || forward_distance > p.max_forward_distance)
                {
                    fail_grab("Approach distance outside calibrated range");
                }
                else
                {
                    publish_mani(object_z + p.lift_offset, p.open_width);
                    change_step(STEP_HAND_UP);
                }
            }
            else
            {
                move_x = std::abs(diff_x) > p.object_tolerance_x ? diff_x : 0.0;
                move_y = std::abs(diff_y) > p.object_tolerance_y ? diff_y : 0.0;
                reset_pose(STEP_ALIGN_RESET);
            }
        }
        else if(grab_step == STEP_ALIGN_RESET || grab_step == STEP_FORWARD_RESET ||
                grab_step == STEP_BACKWARD_RESET)
        {
            if(elapsed > p.motion_timeout)
            {
                fail_grab("Pose reset timed out");
            }
            else if(pose_diff_ready)
            {
                change_step(grab_step == STEP_ALIGN_RESET ? STEP_ALIGN_MOVE :
                            grab_step == STEP_FORWARD_RESET ? STEP_FORWARD_MOVE : STEP_BACKWARD_MOVE);
            }
        }
        else if(grab_step == STEP_ALIGN_MOVE || grab_step == STEP_FORWARD_MOVE ||
                grab_step == STEP_BACKWARD_MOVE)
        {
            if(elapsed > p.motion_timeout)
            {
                fail_grab("Odometry movement timed out");
            }
            else if(!pose_received || tutorial::age(pose_observed) > p.pose_timeout)
            {
                fail_grab("Pose feedback timed out");
            }
            else
            {
                const bool aligning = grab_step == STEP_ALIGN_MOVE;
                const double target_x = aligning ? move_x :
                    grab_step == STEP_FORWARD_MOVE ? forward_distance : -forward_distance;
                const double diff_x = target_x - pose_diff.x;
                const double diff_y = aligning ? move_y - pose_diff.y : 0.0;
                const double speed = aligning ? p.align_speed : p.forward_speed;
                if(std::abs(diff_x) > p.move_tolerance_x)
                {
                    vel_msg.linear.x = diff_x > 0 ? speed : -speed;
                }
                if(aligning && std::abs(diff_y) > p.move_tolerance_y)
                {
                    vel_msg.linear.y = diff_y > 0 ? speed : -speed;
                }
                if(std::abs(diff_x) <= p.move_tolerance_x && std::abs(diff_y) <= p.move_tolerance_y)
                {
                    change_step(aligning ? STEP_ALIGN_STOP :
                                grab_step == STEP_FORWARD_MOVE ? STEP_FORWARD_STOP : STEP_BACKWARD_STOP);
                }
            }
        }
        else if(grab_step == STEP_ALIGN_STOP && elapsed >= p.stop_wait)
        {
            object_valid = false;
            detection_requested = false;
            change_step(STEP_WAIT);
        }
        else if(grab_step == STEP_HAND_UP && elapsed >= p.lift_wait)
        {
            reset_pose(STEP_FORWARD_RESET);
        }
        else if(grab_step == STEP_FORWARD_STOP && elapsed >= p.stop_wait)
        {
            publish_mani(object_z + p.lift_offset, p.grip_width);
            change_step(STEP_GRAB);
        }
        else if(grab_step == STEP_GRAB && elapsed >= p.grip_wait)
        {
            publish_mani(object_z + p.lift_offset + p.raise_height, p.grip_width);
            change_step(STEP_OBJ_UP);
        }
        else if(grab_step == STEP_OBJ_UP && elapsed >= p.raise_wait)
        {
            reset_pose(STEP_BACKWARD_RESET);
        }
        else if(grab_step == STEP_BACKWARD_STOP && elapsed >= p.stop_wait)
        {
            change_step(STEP_DONE);
            tutorial::stop(vel_pub);
            std_msgs::msg::String result_msg;
            result_msg.data = "grab done";
            result_pub->publish(result_msg);
        }

        // 空闲、完成和失败后释放底盘控制；动作等待期间仍可响应停止指令。
        if(active())
        {
            vel_pub->publish(vel_msg);
        }
        loop_rate.sleep();
    }

    publish_command("stop objects");
    if(active())
    {
        tutorial::stop(vel_pub);
    }
    rclcpp::shutdown();
    return grab_step == STEP_FAILED ? 1 : 0;
}
