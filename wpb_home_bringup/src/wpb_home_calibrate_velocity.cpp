/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/
 
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class CVelocityTest : public rclcpp::Node
{
public:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr odom_ctrl_pub;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_diff_sub;
  geometry_msgs::msg::Pose2D pose_diff;

  CVelocityTest() : Node("wpb_home_calibrate_velocity")
  {
    // Set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    odom_ctrl_pub = this->create_publisher<std_msgs::msg::String>("/wpb_home/ctrl", 30);
    pose_diff_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "/wpb_home/pose_diff", 1, std::bind(&CVelocityTest::PoseDiffCallback, this, _1));
  }

  void PoseDiffCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    pose_diff.x = msg->x;
    pose_diff.y = msg->y;
    pose_diff.theta = msg->theta;
  }

  bool driveKeyboard()
  {
    std::cout << "输入如下指令并回车执行：\n"
      "W - 前进1米 \nS - 后退1米\n"
      "A - 左平移1米\nD - 右平移1米 \n"
      "Q - 逆时针转360度\nE - 顺时针转360度\nX - 退出\n";

    //we will be sending commands of type "twist"
    geometry_msgs::msg::Twist base_cmd;
    rclcpp::Rate r(2);

    char cmd[50];
    while(rclcpp::ok())
    {
      std::cin.getline(cmd, 50);
      if(cmd[0]!='w' && cmd[0]!='a' && cmd[0]!='s' && cmd[0]!='d' && cmd[0]!='q' && cmd[0]!='e' && cmd[0]!='x')
      {
        std::cout << "无效指令：" << cmd << "\n";
        continue;
      }
      else
      {
        std_msgs::msg::String odom_ctrl_msg;
        odom_ctrl_msg.data = "pose_diff reset";
        odom_ctrl_pub->publish(odom_ctrl_msg);
        // rclcpp::spin_some(this);
        std::cout << "准备执行指令：" << cmd << "\n";
        r.sleep();
      }

      int nDelay = 0;
      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
      //move forward
      if(cmd[0]=='w')
      {
        base_cmd.linear.x = 0.25;
        nDelay = 4;
      } 
      if(cmd[0]=='s')
      {
        base_cmd.linear.x = -0.25;
        nDelay = 4;
      } 
      //shift left 
      else if(cmd[0]=='a')
      {
        base_cmd.linear.y = 0.25;
        nDelay = 4;
      
      } 
      //shift right 
      else if(cmd[0]=='d')
      {
        base_cmd.linear.y = -0.25;
        nDelay = 4;
      } 
      //turn left (yaw) 
      else if(cmd[0]=='q')
      {
        base_cmd.angular.z = 3.1415926/4; 
        nDelay = 8;
      } 
      //turn right (yaw) 
      else if(cmd[0]=='e')
      {
        base_cmd.angular.z = -3.1415926/4;
        nDelay = 8;
      } 
      //quit
      else if(cmd[0]=='x')
      {
        break;
      }

      while(nDelay > 0)
      {
          std::cout << "倒计时 = " << nDelay << "\n";
          nDelay --;
          //publish the assembled command
          cmd_vel_pub_->publish(base_cmd);
          sleep(1);     
      }
      
      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;
      cmd_vel_pub_->publish(base_cmd);
      // rclcpp::spin_some(this);
      r.sleep();
      
      std::cout << "任务完成!\n";
      std::cout << "-------------------------\n";
      printf("**电机码盘里程计**\n前后运动= %.2f 米\n左右平移= %.2f 米\n转动角度= %.2f 度\n",pose_diff.x,pose_diff.y,pose_diff.theta*180/3.1415926);
      std_msgs::msg::String odom_ctrl_msg;
      odom_ctrl_msg.data = "motor encoder";
      odom_ctrl_pub->publish(odom_ctrl_msg);
      // rclcpp::spin_some(this);
      std::cout << "-------------------------\n";
      std::cout << "\n";
      std::cout << "输入下一条指令：";
    }
    return true;
  }

  // ... (The rest of the class remains the same, with necessary adaptations for ROS2)
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CVelocityTest>();
  node->driveKeyboard();
  rclcpp::shutdown();
  return 0;
}


