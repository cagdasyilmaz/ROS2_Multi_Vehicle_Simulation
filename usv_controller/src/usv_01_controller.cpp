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
    	// Access subscriber node functionalities or data
    	// For example, you can call functions or access data members of the subscriber node
    	// subscriber_node_->some_function();
    	// auto data = subscriber_node_->get_data();

    	auto position = subscriber_node_->get_position();

    	if(position.x < 10.0f)
    		message_.linear.x = 0.10;
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

    auto node_pub = std::make_shared<PublisherNode>("usv_01_controller_pub", fleet_info[0], "/box_bot1/cmd_vel");

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

        printf("[Boat1] [x, y]: [%.2f, %.2f]\n", vehicle_positions[0].x, vehicle_positions[0].y);


        std::this_thread::sleep_for(200ms); // Add a small delay to reduce CPU usage
    }

    rclcpp::shutdown();
    return 0;
}
