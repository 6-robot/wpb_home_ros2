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
#include "driver/WPB_Home_driver.h"

using namespace std::chrono_literals;

#define TEST_READY    0
#define TEST_MOTOR1_F 1
#define TEST_MOTOR1_N 2
#define TEST_MOTOR2_F 3
#define TEST_MOTOR2_N 4
#define TEST_MOTOR3_F 5
#define TEST_MOTOR3_N 6

class WPBHomeTestMotors : public rclcpp::Node
{
public:
    WPBHomeTestMotors() : Node("wpb_home_test_motors"), nTest(TEST_READY)
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        std::string strSerialPort;
        this->get_parameter("serial_port", strSerialPort);
        m_wpb_home.Open(strSerialPort.c_str(), 115200);

        timer_ = this->create_wall_timer(3333ms, std::bind(&WPBHomeTestMotors::testMotors, this));
    }

private:
    void testMotors()
    {
        int nMotorSpeed = 3000;
        switch (nTest)
        {
        case TEST_READY:
            RCLCPP_INFO(this->get_logger(), "TEST_READY");
            m_wpb_home.SendMotors(0, 0, 0, 0);
            nTest++;
            break;
        case TEST_MOTOR1_F:
            RCLCPP_INFO(this->get_logger(), "TEST_MOTOR1_F");
            m_wpb_home.SendMotors(nMotorSpeed, 0, 0, 0);
            nTest++;
            break;
        case TEST_MOTOR1_N:
            RCLCPP_INFO(this->get_logger(), "TEST_MOTOR1_N");
            m_wpb_home.SendMotors(-nMotorSpeed, 0, 0, 0);
            nTest++;
            break;
        case TEST_MOTOR2_F:
            RCLCPP_INFO(this->get_logger(), "TEST_MOTOR2_F");
            m_wpb_home.SendMotors(0, nMotorSpeed, 0, 0);
            nTest++;
            break;
        case TEST_MOTOR2_N:
            RCLCPP_INFO(this->get_logger(), "TEST_MOTOR2_N");
            m_wpb_home.SendMotors(0, -nMotorSpeed, 0, 0);
            nTest++;
            break; 
        case TEST_MOTOR3_F:
            RCLCPP_INFO(this->get_logger(), "TEST_MOTOR3_F");
            m_wpb_home.SendMotors(0, 0, nMotorSpeed, 0);
            nTest++;
            break;
        case TEST_MOTOR3_N:
            RCLCPP_INFO(this->get_logger(), "TEST_MOTOR3_N");
            m_wpb_home.SendMotors(0, 0, -nMotorSpeed, 0);
            nTest++;
            break;
        default:
            nTest = TEST_READY;
            m_wpb_home.SendMotors(0, 0, 0, 0);
            break;
        }
    }

    CWPB_Home_driver m_wpb_home;
    int nTest;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WPBHomeTestMotors>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
