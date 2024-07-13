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
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

class CWPHLidarFilter : public rclcpp::Node
{
public:
    CWPHLidarFilter(const std::string & topic_name);

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    std::string pub_topic_;
};

CWPHLidarFilter::CWPHLidarFilter(const std::string & topic_name) : Node("wpb_home_lidar_filter"), pub_topic_(topic_name)
{
    std::string pub_topic_;
    this->declare_parameter<std::string>("pub_topic", "scan");
    this->get_parameter("pub_topic", pub_topic_);
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_topic_, 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_raw", 10, std::bind(&CWPHLidarFilter::lidarCallback, this, _1));
}

void CWPHLidarFilter::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    int nRanges = scan->ranges.size();
    float k = nRanges / 360.0f;
    auto new_scan = sensor_msgs::msg::LaserScan();
    new_scan.header.stamp = scan->header.stamp;
    new_scan.header.frame_id = scan->header.frame_id;
    new_scan.angle_max = scan->angle_max;
    new_scan.angle_min = scan->angle_min;
    new_scan.angle_increment = M_PI / 180;
    new_scan.time_increment = scan->time_increment * k;
    new_scan.range_min = 0.25;
    new_scan.range_max = scan->range_max;
    new_scan.ranges.resize(360);
    new_scan.intensities.resize(360);
    for (int i = 0; i < 360; i++)
    {
        new_scan.ranges[i] = scan->ranges[static_cast<int>(i * k)];
        if (new_scan.ranges[i] < 0.25)
        {
            new_scan.ranges[i] = new_scan.range_max + 1.0;
        }
        new_scan.intensities[i] = scan->intensities[static_cast<int>(i * k)];
    }
    scan_pub_->publish(new_scan);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CWPHLidarFilter>("/scan");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}