#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "../include/subscriptions/odometry.hpp"

#include <memory>

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

    	if(position.x > 10.5f)
    		message_.linear.x = -0.10;
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

    auto node_sub = std::make_shared<SubscriberNode>("usv_06_controller_sub", "/box_bot6/odom");
    auto node_pub = std::make_shared<PublisherNode>("usv_06_controller_pub", node_sub, "/box_bot6/cmd_vel");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_sub);
    executor.add_node(node_pub);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
