#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "driver/WPB_Home_driver.h"

std::shared_ptr<rclcpp::Node> node;
static CWPB_Home_driver m_wpb_home;
static int nLastMotorPos[3];

static float fKVx = 1.0f/sqrt(3.0f);
static float fKVy = 2.0f/3.0f;
static float fKVz = 1.0f/3.0f;
static geometry_msgs::msg::Pose2D lastPose;
static geometry_msgs::msg::Twist lastVel;
static geometry_msgs::msg::Pose2D pose_diff_msg;

void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
//   RCLCPP_INFO(node->get_logger(), "liner(%.2f %.2f) angular(%.2f)", msg->linear.x, msg->linear.y, msg->angular.z);
  m_wpb_home.Velocity(msg->linear.x, msg->linear.y, msg->angular.z);
}

static float kForearm = 1.57/11200;
static float fLiftValue = 0;
static float fLiftVelocity = 0;
static float fGripperValue = 0;
static float fGripperVelocity = 0;
void ManiCtrlCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // int nNumJoint = msg->position.size();
  // for(int i=0;i<nNumJoint;i++)
  // {
  //     RCLCPP_INFO(node->get_logger(), "%d - %s = %.2f  vel= %.2f", i, msg->name[i].c_str(),msg->position[i],msg->velocity[i]);
  // }
  // 高度升降
  fLiftValue = msg->position[0];
  fLiftVelocity = msg->velocity[0];
  // 手爪
  fGripperValue = msg->position[1];
  fGripperVelocity = msg->velocity[1];

  m_wpb_home.ManiCmd(fLiftValue, fLiftVelocity, fGripperValue, fGripperVelocity);
}

void CtrlCallback(const std_msgs::msg::String::SharedPtr msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("pose_diff reset");
    if( nFindIndex >= 0 )
    {
        pose_diff_msg.x = 0;
        pose_diff_msg.y = 0;
        pose_diff_msg.theta = 0;
        //RCLCPP_INFO(node->get_logger(),"[pose_diff reset]");
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("wpb_home_core");

    // 参数读取
    std::string strSerialPort;
    node->declare_parameter<std::string>("serial_port", "/dev/ftdi");
    node->get_parameter("serial_port", strSerialPort);
    m_wpb_home.Open(strSerialPort.c_str(),115200);
    RCLCPP_INFO(node->get_logger(), "机器人底盘端口 = %s", strSerialPort.c_str());

    bool bOdom = true;
    node->get_parameter_or("odom", bOdom, true);

    // 计时变量
    rclcpp::Time current_time = node->now();
    rclcpp::Time last_time = node->now();
    rclcpp::Rate loop_rate(100);

    // 机械臂 joint_states 初始化
    auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 100);
    sensor_msgs::msg::JointState joint_msg;
    std::vector<std::string> joint_name(6);
    std::vector<double> joint_pos(6);
    joint_name[0] = "mani_base";
    joint_name[1] = "elbow_forearm";
    joint_name[2] = "forearm_left_finger";
    joint_name[3] = "forearm_right_finger";
    joint_name[4] = "kinect_height";
    joint_name[5] = "kinect_pitch";
    joint_pos[0] = 0.0;
    joint_pos[1] = 0.0;
    joint_pos[2] = 0.0;
    joint_pos[3] = 0.0;
    joint_pos[4] = 0.0;
    joint_pos[5] = 0.0;
    node->declare_parameter<float>("kinect_height", 0.0);
    node->get_parameter("kinect_height", joint_pos[4]);
    node->declare_parameter<float>("kinect_pitch", 0.0);
    node->get_parameter("kinect_pitch", joint_pos[5]);
    joint_msg.name = joint_name;
    joint_msg.position = joint_pos;
    RCLCPP_WARN(node->get_logger(), "kinect_height = %.2f", joint_pos[4]);
    RCLCPP_WARN(node->get_logger(), "kinect_pitch = %.2f", joint_pos[5]);

    // 里程计初始化
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    geometry_msgs::msg::TransformStamped odom_trans;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::Quaternion odom_quat;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    tf2::Quaternion odom_q;
    odom_q.setRPY(0, 0, 0);
    odom_trans.transform.rotation.x = odom_q.x();
    odom_trans.transform.rotation.y = odom_q.y();
    odom_trans.transform.rotation.z = odom_q.z();
    odom_trans.transform.rotation.w = odom_q.w();
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0.0;
    odom_quat.w = 1.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;

    // 位移机制
    auto ctrl_sub = node->create_subscription<std_msgs::msg::String>("/wpb_home/ctrl", 10, CtrlCallback);
    auto pose_diff_pub = node->create_publisher<geometry_msgs::msg::Pose2D>("/wpb_home/pose_diff", 1);
    pose_diff_msg.x = 0;
    pose_diff_msg.y = 0;
    pose_diff_msg.theta = 0;
    lastPose.x = lastPose.y = lastPose.theta = 0;
    lastVel.linear.x = lastVel.linear.y = lastVel.linear.z = lastVel.angular.x = lastVel.angular.y = lastVel.angular.z = 0;
    nLastMotorPos[0] = nLastMotorPos[1] = nLastMotorPos[2] = 0;

    // IMU
    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    // 底盘运动
    auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, CmdVelCallback);
    
    // 手臂控制
    auto mani_ctrl_sub = node->create_subscription<sensor_msgs::msg::JointState>("/wpb_home/mani_ctrl", 10, ManiCtrlCallback);
    
    while (rclcpp::ok())
    {
        m_wpb_home.ReadNewData();
        m_wpb_home.nParseCount ++;
        //ROS_INFO("[m_wpb_home.nParseCount]= %d",m_wpb_home.nParseCount);
        if(m_wpb_home.nParseCount > 100)
        {
            m_wpb_home.arMotorPos[0] =0; nLastMotorPos[0] = 0;
            m_wpb_home.arMotorPos[1] =0; nLastMotorPos[1] = 0;
            m_wpb_home.arMotorPos[2] =0; nLastMotorPos[2] = 0;
            m_wpb_home.nParseCount = 0;
            //ROS_INFO("empty");
        }

        last_time = current_time;
        current_time = node->now();

        // 里程计发布
        double fVx,fVy,fVz;
        double fPosDiff[3];
        if(nLastMotorPos[0] != m_wpb_home.arMotorPos[0] || nLastMotorPos[1] != m_wpb_home.arMotorPos[1] || nLastMotorPos[2] != m_wpb_home.arMotorPos[2])
        {
            fPosDiff[0] = (double)(m_wpb_home.arMotorPos[0] - nLastMotorPos[0]); 
            fPosDiff[1] = (double)(m_wpb_home.arMotorPos[1] - nLastMotorPos[1]);
            fPosDiff[2] = (double)(m_wpb_home.arMotorPos[2] - nLastMotorPos[2]);
            
            fVx = (fPosDiff[1] - fPosDiff[0]) * fKVx;
            fVy = (fPosDiff[0] + fPosDiff[1]) - (fPosDiff[0] + fPosDiff[1] + fPosDiff[2])*fKVy;
            fVz = (fPosDiff[0] + fPosDiff[1] + fPosDiff[2])*fKVz;
            double fTimeDur = (current_time - last_time).seconds();
            fVx = fVx/(fTimeDur*9100);
            fVy = fVy/(fTimeDur*9100);
            fVz = fVz/(fTimeDur*1840);
            
            double dx = (lastVel.linear.x*cos(lastPose.theta) - lastVel.linear.y*sin(lastPose.theta))*fTimeDur;
            double dy = (lastVel.linear.x*sin(lastPose.theta) + lastVel.linear.y*cos(lastPose.theta))*fTimeDur;

            lastPose.x += dx;
            lastPose.y += dy;
            lastPose.theta += (fVz*fTimeDur);

            double pd_dx = (lastVel.linear.x*cos(pose_diff_msg.theta) - lastVel.linear.y*sin(pose_diff_msg.theta))*fTimeDur;
            double pd_dy = (lastVel.linear.x*sin(pose_diff_msg.theta) + lastVel.linear.y*cos(pose_diff_msg.theta))*fTimeDur;
            pose_diff_msg.x += pd_dx;
            pose_diff_msg.y += pd_dy;
            pose_diff_msg.theta += (fVz*fTimeDur);

            tf2::Quaternion tf_quat;
            tf_quat.setRPY(0, 0, lastPose.theta);
            odom_quat.x = tf_quat.x();
            odom_quat.y = tf_quat.y();
            odom_quat.z = tf_quat.z();
            odom_quat.w = tf_quat.w();
            //updata transform
            odom_trans.header.stamp = current_time;
            odom_trans.transform.translation.x = lastPose.x;
            odom_trans.transform.translation.y = lastPose.y;
            odom_trans.transform.translation.z = 0;
            odom_trans.transform.rotation.x = tf_quat.x();
            odom_trans.transform.rotation.y = tf_quat.y();
            odom_trans.transform.rotation.z = tf_quat.z();
            odom_trans.transform.rotation.w = tf_quat.w();

            //filling the odometry
            odom.header.stamp = current_time;
            //position
            odom.pose.pose.position.x = lastPose.x;
            odom.pose.pose.position.y = lastPose.y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;
            //velocity
            odom.twist.twist.linear.x = fVx;
            odom.twist.twist.linear.y = fVy;
            odom.twist.twist.linear.z = 0;
            odom.twist.twist.angular.x = 0;
            odom.twist.twist.angular.y = 0;
            odom.twist.twist.angular.z = fVz;

            if(bOdom == true)
            {
                //plublishing the odometry and new tf
                broadcaster->sendTransform(odom_trans);
                odom_pub->publish(odom);
            }

            lastVel.linear.x = fVx;
            lastVel.linear.y = fVy;
            lastVel.angular.z = fVz;

            nLastMotorPos[0] = m_wpb_home.arMotorPos[0];
            nLastMotorPos[1] = m_wpb_home.arMotorPos[1];
            nLastMotorPos[2] = m_wpb_home.arMotorPos[2];
        }
        else
        {
            if(bOdom == true)
            {
                odom_trans.header.stamp = node->now();
                //plublishing the odometry and new tf
                broadcaster->sendTransform(odom_trans);
                odom.header.stamp = node->now();
                odom_pub->publish(odom);
                //ROS_INFO("[odom] zero");
            }
        }
        pose_diff_pub->publish(pose_diff_msg);
        //ROS_INFO("[pose_diff_msg] x= %.2f  y=%.2f  th= %.2f", pose_diff_msg.x,pose_diff_msg.y,pose_diff_msg.theta);
        
        // IMU 发布
        sensor_msgs::msg::Imu imu_msg;	
        imu_msg.header.stamp = node->now();
        imu_msg.header.frame_id = "imu";
        imu_msg.orientation.x = m_wpb_home.fQuatX;
        imu_msg.orientation.y = m_wpb_home.fQuatY;
        imu_msg.orientation.z = m_wpb_home.fQuatZ;
        imu_msg.orientation.w = m_wpb_home.fQuatW;

        imu_msg.angular_velocity.x = m_wpb_home.fGyroX;
        imu_msg.angular_velocity.y = m_wpb_home.fGyroY;
        imu_msg.angular_velocity.z = m_wpb_home.fGyroZ;

        imu_msg.linear_acceleration.x = m_wpb_home.fAccX;
        imu_msg.linear_acceleration.y = m_wpb_home.fAccY;
        imu_msg.linear_acceleration.z = m_wpb_home.fAccZ;

        imu_pub->publish(imu_msg);

        // 发布手臂TF
        joint_msg.header.stamp = node->now();
        // joint_msg.header.seq++;
        joint_pos[0] = m_wpb_home.arMotorPos[4] * 0.00001304;
        if (m_wpb_home.arMotorPos[4] < 11200)
        {
            joint_pos[1] = m_wpb_home.arMotorPos[4] * kForearm;
        }
        else
        {
            joint_pos[1] = 1.57;
        }
        joint_pos[2] = (47998 - m_wpb_home.arMotorPos[5]) * 0.00001635;
        joint_pos[3] = joint_pos[2];
        joint_msg.name = joint_name;
        joint_msg.position = joint_pos;

        joint_state_pub->publish(joint_msg);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}