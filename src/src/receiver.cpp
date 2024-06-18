#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "pickingPoint/coordinates", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::Int32MultiArray &msg) const
    {
        RCLCPP_INFO(this->get_logger(), "(%d, %d)", msg.data[0], msg.data[1]);
    }
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}