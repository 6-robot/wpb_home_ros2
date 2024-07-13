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
#include "driver/WPB_Home_driver.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class WPBHomeMotorEncoder : public rclcpp::Node
{
public:
    WPBHomeMotorEncoder() : Node("wpb_home_motor_encoder"), vel_x(0.3), vel_y(0), vel_z(0), bFirst(true)
    {
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, std::bind(&WPBHomeMotorEncoder::cmdVelCallback, this, _1));

        // 初始化驱动，假设WPB_Home_driver已经被适配为ROS2
        m_wpb_home.Open("/dev/ftdi", 115200);

        timer_ = this->create_wall_timer(10ms, std::bind(&WPBHomeMotorEncoder::update, this));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        vel_x = msg->linear.x;
        vel_y = msg->linear.y;
        vel_z = msg->angular.z;
    }

    void update()
    {
        m_wpb_home.ReadNewData();

        display_count++;
        if (display_count > 100)
        {
            display_count = 0;
            RCLCPP_WARN(this->get_logger(), " [M1] %d   [M2] %d   [M3] %d", 
                m_wpb_home.arMotorPos[0], m_wpb_home.arMotorPos[1], m_wpb_home.arMotorPos[2]);
            m_wpb_home.Velocity(vel_x, vel_y, vel_z);
        }

        if (bFirst == true)
        {
            nFirstVal[0] = m_wpb_home.arMotorPos[0];
            nFirstVal[1] = m_wpb_home.arMotorPos[1];
            nFirstVal[2] = m_wpb_home.arMotorPos[2];
            bFirst = false;
        }
        else
        {
            int nDiff[3];
            nDiff[0] = m_wpb_home.arMotorPos[0] - nFirstVal[0];
            nDiff[1] = m_wpb_home.arMotorPos[1] - nFirstVal[1];
            nDiff[2] = m_wpb_home.arMotorPos[2] - nFirstVal[2];
            if(false)
                RCLCPP_WARN(this->get_logger(), " [M1_diff] %d   [M2_diff] %d   [M3_diff] %d",nDiff[0],nDiff[1],nDiff[2]);
        }
    }

    CWPB_Home_driver m_wpb_home;
    float vel_x;
    float vel_y;
    float vel_z;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    int nFirstVal[3];
    bool bFirst;
    int display_count = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WPBHomeMotorEncoder>());
    rclcpp::shutdown();
    return 0;
}
