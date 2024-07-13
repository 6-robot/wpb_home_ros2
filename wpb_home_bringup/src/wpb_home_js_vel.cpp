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
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class TeleopJoy : public rclcpp::Node
{
public:
  TeleopJoy();
  void send_vel_cmd();

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy);
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velcmd_pub_;
  bool bStart;
  float lx;
  float ly;
  float ry;
};

TeleopJoy::TeleopJoy()
: Node("teleop_joy_node"), bStart(false), lx(0), ly(0), ry(0)
{
  velcmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&TeleopJoy::joy_callback, this, _1));
  
  RCLCPP_INFO(this->get_logger(), "TeleopJoy Node Started");
}

void TeleopJoy::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  lx = joy->axes[1];  // forward & backward
  ly = joy->axes[0];  // shift
  ry = joy->axes[3];  // rotation
  
  bStart = true;
}

void TeleopJoy::send_vel_cmd()
{
  if (!bStart)
    return;

  auto vel_cmd = geometry_msgs::msg::Twist();
  vel_cmd.linear.x = lx * 0.2;
  vel_cmd.linear.y = ly * 0.2;
  vel_cmd.angular.z = ry * 0.5;

  velcmd_pub_->publish(vel_cmd);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopJoy>();
  rclcpp::Rate r(30);
  while (rclcpp::ok()) {
    node->send_vel_cmd();
    rclcpp::spin_some(node);
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
