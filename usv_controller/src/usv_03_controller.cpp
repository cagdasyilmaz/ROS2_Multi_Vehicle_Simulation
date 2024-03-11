/****************************************************************************
 * MIT License
 *
 * Copyright (c) 2024 İsmail Çağdaş Yılmaz
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ****************************************************************************/

#include "../include/subscriptions/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PublisherNode : public rclcpp::Node
{
public:
	PublisherNode(const std::string& node_name, const std::shared_ptr<SubscriberNode>& subscriber_node, const std::string& topic_name)
    	: Node(node_name), subscriber_node_(subscriber_node)
    {
		publisher_ = create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);
        timer_ = create_wall_timer(50ms, std::bind(&PublisherNode::timer_callback, this));
    }

    void timer_callback()
    {
    	auto position = subscriber_node_->get_position();

    	if(position.x < 10.0f)
    		message_.linear.x = 0.15;
    	else
    		message_.linear.x = 0.0;

    	message_.linear.y  = 0.0;
        message_.linear.z  = 0.0;
    	message_.angular.x = 0.0;
    	message_.angular.y = 0.0;
    	message_.angular.z = 0.0;
    	publisher_->publish(message_);
    }

private:
    std::shared_ptr<SubscriberNode> subscriber_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist message_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::vector<std::shared_ptr<SubscriberNode>> fleet_info;

    fleet_info.push_back(std::make_shared<SubscriberNode>("usv_01_controller_sub", "/box_bot1/odom"));
    fleet_info.push_back(std::make_shared<SubscriberNode>("usv_02_controller_sub", "/box_bot2/odom"));
    fleet_info.push_back(std::make_shared<SubscriberNode>("usv_03_controller_sub", "/box_bot3/odom"));
    fleet_info.push_back(std::make_shared<SubscriberNode>("usv_04_controller_sub", "/box_bot4/odom"));
    fleet_info.push_back(std::make_shared<SubscriberNode>("usv_05_controller_sub", "/box_bot5/odom"));
    fleet_info.push_back(std::make_shared<SubscriberNode>("usv_06_controller_sub", "/box_bot6/odom"));

    auto node_pub = std::make_shared<PublisherNode>("usv_03_controller_pub", fleet_info[2], "/box_bot3/cmd_vel");

    // Manually spinning nodes with a while loop
    while (rclcpp::ok())
    {

    	for(auto vehicle_info : fleet_info)
    	{
    		rclcpp::spin_some(vehicle_info);
    	}
    	rclcpp::spin_some(node_pub);

    	std::vector<Position> vehicle_positions;

    	for(auto vehicle_info : fleet_info)
    	{
    		vehicle_positions.push_back(vehicle_info->get_position());
    	}

        //printf("[Boat3] [x, y]: [%.2f, %.2f]\n", vehicle_positions[2].x, vehicle_positions[2].y);

        std::this_thread::sleep_for(200ms); // Add a small delay to reduce CPU usage
    }

    rclcpp::shutdown();
    return 0;
}
