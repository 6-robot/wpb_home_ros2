#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wpr_simulation2/msg/object.hpp>
#include <chrono>
#include <cmath>

#define STEP_WAIT           0
#define STEP_ALIGN_OBJ      1
#define STEP_HAND_UP       2
#define STEP_FORWARD       3
#define STEP_GRAB           4
#define STEP_OBJ_UP          5
#define STEP_BACKWARD     6
#define STEP_DONE           7
#define STEP_ALIGN_RESET    8
#define STEP_ALIGN_MOVE     9
#define STEP_ALIGN_STOP     10
#define STEP_FORWARD_RESET  11
#define STEP_FORWARD_MOVE   12
#define STEP_FORWARD_STOP   13
#define STEP_BACKWARD_RESET 14
#define STEP_BACKWARD_MOVE  15
#define STEP_BACKWARD_STOP  16
static int grab_step = STEP_WAIT;

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mani_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ctrl_pub;

double object_x = 0.0;
double object_y = 0.0;
double object_z = 0.0;

const double align_x = 1.0;
const double align_y = 0.0;
const double object_tolerance_x = 0.03;
const double object_tolerance_y = 0.03;
const double move_tolerance_x = 0.02;
const double move_tolerance_y = 0.01;
const double align_speed = 0.05;
const double forward_speed = 0.1;
const double reset_wait = 0.5;
const double stop_wait = 0.5;

double move_x = 0.0;
double move_y = 0.0;
double forward_distance = 0.0;
geometry_msgs::msg::Pose2D pose_diff;
bool pose_diff_ready = false;
bool detection_requested = false;
rclcpp::Time detection_start;
std::chrono::steady_clock::time_point align_time;

void PoseDiffCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    pose_diff = *msg;
    if((grab_step == STEP_ALIGN_RESET || grab_step == STEP_FORWARD_RESET ||
        grab_step == STEP_BACKWARD_RESET) &&
       std::chrono::duration<double>(std::chrono::steady_clock::now() - align_time).count() >= reset_wait &&
       std::fabs(pose_diff.x) <= 0.001 &&
       std::fabs(pose_diff.y) <= 0.001 &&
       std::fabs(pose_diff.theta) <= 0.001)
    {
        pose_diff_ready = true;
    }
}

void ObjectCallback(const wpr_simulation2::msg::Object::SharedPtr msg)
{
    if(grab_step == STEP_WAIT && detection_requested &&
       !msg->x.empty() && !msg->y.empty() && !msg->z.empty() &&
       rclcpp::Time(msg->header.stamp, node->get_clock()->get_clock_type()) >= detection_start)
    {
        object_x = msg->x[0];
        object_y = msg->y[0];
        object_z = msg->z[0];
        grab_step = STEP_ALIGN_OBJ;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("grab_node");

    cmd_pub = node->create_publisher<std_msgs::msg::String>(
        "/wpb_home/behavior", 
        10
    );
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 
        10
    );
    mani_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        "/wpb_home/mani_ctrl",
         10
    );
    ctrl_pub = node->create_publisher<std_msgs::msg::String>(
        "/wpb_home/ctrl", 10
    );
    auto pose_diff_sub = node->create_subscription<geometry_msgs::msg::Pose2D>(
        "/wpb_home/pose_diff", 1, PoseDiffCallback
    );
    auto object_sub = node->create_subscription<wpr_simulation2::msg::Object>(
        "/wpb_home/objects_3d", 
        10, 
        ObjectCallback
    );
    
    rclcpp::Rate loop_rate(30);

    while(rclcpp::ok())
    {
        if(grab_step == STEP_WAIT)
        {
            if(!detection_requested && cmd_pub->get_subscription_count() > 0)
            {
                std_msgs::msg::String start_msg;
                start_msg.data = "start objects";
                detection_start = node->now();
                cmd_pub->publish(start_msg);
                detection_requested = true;
            }
        }
        else if(grab_step == STEP_ALIGN_OBJ)
        {
            double diff_x = object_x - align_x;
            double diff_y = object_y - align_y;
            std_msgs::msg::String stop_msg;
            stop_msg.data = "stop objects";
            cmd_pub->publish(stop_msg);
            vel_pub->publish(geometry_msgs::msg::Twist());

            if(std::fabs(diff_x) <= object_tolerance_x &&
               std::fabs(diff_y) <= object_tolerance_y)
            {
                grab_step = STEP_HAND_UP;
            }
            else
            {
                move_x = std::fabs(diff_x) > object_tolerance_x ? diff_x : 0.0;
                move_y = std::fabs(diff_y) > object_tolerance_y ? diff_y : 0.0;
                pose_diff_ready = false;
                align_time = std::chrono::steady_clock::now();
                grab_step = STEP_ALIGN_RESET;
                std_msgs::msg::String reset_msg;
                reset_msg.data = "pose_diff reset";
                ctrl_pub->publish(reset_msg);
                RCLCPP_INFO(node->get_logger(), "[STEP_ALIGN_RESET] move = (%.3f, %.3f)", move_x, move_y);
            }
        }
        else if(grab_step == STEP_ALIGN_RESET)
        {
            vel_pub->publish(geometry_msgs::msg::Twist());
            if(pose_diff_ready)
            {
                grab_step = STEP_ALIGN_MOVE;
            }
        }
        else if(grab_step == STEP_ALIGN_MOVE)
        {
            double diff_x = move_x - pose_diff.x;
            double diff_y = move_y - pose_diff.y;
            geometry_msgs::msg::Twist vel_msg;
            if(std::fabs(diff_x) > move_tolerance_x)
            {
                vel_msg.linear.x = diff_x > 0 ? align_speed : -align_speed;
            }
            if(std::fabs(diff_y) > move_tolerance_y)
            {
                vel_msg.linear.y = diff_y > 0 ? align_speed : -align_speed;
            }
            vel_pub->publish(vel_msg);
            if(std::fabs(diff_x) <= move_tolerance_x &&
               std::fabs(diff_y) <= move_tolerance_y)
            {
                align_time = std::chrono::steady_clock::now();
                grab_step = STEP_ALIGN_STOP;
            }
        }
        else if(grab_step == STEP_ALIGN_STOP)
        {
            vel_pub->publish(geometry_msgs::msg::Twist());
            if(std::chrono::duration<double>(std::chrono::steady_clock::now() - align_time).count() >= stop_wait)
            {
                detection_requested = false;
                grab_step = STEP_WAIT;
            }
        }
        if(grab_step == STEP_HAND_UP)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_HAND_UP]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z + 0.04;
            mani_msg.position[1] = 0.15;
            mani_pub->publish(mani_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(18000));
            grab_step = STEP_FORWARD;
        }
        if(grab_step == STEP_FORWARD)
        {
            forward_distance = 0.7 * (object_x - 0.7);
            vel_pub->publish(geometry_msgs::msg::Twist());
            pose_diff_ready = false;
            align_time = std::chrono::steady_clock::now();
            grab_step = STEP_FORWARD_RESET;
            std_msgs::msg::String reset_msg;
            reset_msg.data = "pose_diff reset";
            ctrl_pub->publish(reset_msg);
            RCLCPP_INFO(node->get_logger(), "[STEP_FORWARD_RESET] distance = %.3f", forward_distance);
        }
        else if(grab_step == STEP_FORWARD_RESET)
        {
            vel_pub->publish(geometry_msgs::msg::Twist());
            if(pose_diff_ready)
            {
                grab_step = STEP_FORWARD_MOVE;
            }
        }
        else if(grab_step == STEP_FORWARD_MOVE)
        {
            double diff_x = forward_distance - pose_diff.x;
            geometry_msgs::msg::Twist vel_msg;
            if(std::fabs(diff_x) > move_tolerance_x)
            {
                vel_msg.linear.x = diff_x > 0 ? forward_speed : -forward_speed;
            }
            vel_pub->publish(vel_msg);
            if(std::fabs(diff_x) <= move_tolerance_x)
            {
                align_time = std::chrono::steady_clock::now();
                grab_step = STEP_FORWARD_STOP;
            }
        }
        else if(grab_step == STEP_FORWARD_STOP)
        {
            vel_pub->publish(geometry_msgs::msg::Twist());
            if(std::chrono::duration<double>(std::chrono::steady_clock::now() - align_time).count() >= stop_wait)
            {
                grab_step = STEP_GRAB;
            }
        }
        if(grab_step == STEP_GRAB)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_GRAB]");
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_pub->publish(vel_msg);
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z +0.04;
            mani_msg.position[1] = 0.045;
            mani_pub->publish(mani_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            grab_step = STEP_OBJ_UP;
        }
        if(grab_step == STEP_OBJ_UP)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_OBJ_UP]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z + 0.09;
            mani_msg.position[1] = 0.045;
            mani_pub->publish(mani_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            grab_step = STEP_BACKWARD;
        }
        if(grab_step == STEP_BACKWARD)
        {
            vel_pub->publish(geometry_msgs::msg::Twist());
            pose_diff_ready = false;
            align_time = std::chrono::steady_clock::now();
            grab_step = STEP_BACKWARD_RESET;
            std_msgs::msg::String reset_msg;
            reset_msg.data = "pose_diff reset";
            ctrl_pub->publish(reset_msg);
            RCLCPP_INFO(node->get_logger(), "[STEP_BACKWARD_RESET] distance = %.3f", -forward_distance);
        }
        else if(grab_step == STEP_BACKWARD_RESET)
        {
            vel_pub->publish(geometry_msgs::msg::Twist());
            if(pose_diff_ready)
            {
                grab_step = STEP_BACKWARD_MOVE;
            }
        }
        else if(grab_step == STEP_BACKWARD_MOVE)
        {
            double diff_x = -forward_distance - pose_diff.x;
            geometry_msgs::msg::Twist vel_msg;
            if(std::fabs(diff_x) > move_tolerance_x)
            {
                vel_msg.linear.x = diff_x > 0 ? forward_speed : -forward_speed;
            }
            vel_pub->publish(vel_msg);
            if(std::fabs(diff_x) <= move_tolerance_x)
            {
                align_time = std::chrono::steady_clock::now();
                grab_step = STEP_BACKWARD_STOP;
            }
        }
        else if(grab_step == STEP_BACKWARD_STOP)
        {
            vel_pub->publish(geometry_msgs::msg::Twist());
            if(std::chrono::duration<double>(std::chrono::steady_clock::now() - align_time).count() >= stop_wait)
            {
                grab_step = STEP_DONE;
                RCLCPP_INFO(node->get_logger(), "[STEP_DONE]");
            }
        }
        if(grab_step == STEP_DONE)
        {
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_pub->publish(vel_msg);
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
