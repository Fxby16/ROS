#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <memory>

class Receiver : public rclcpp::Node
{
public:
    Receiver()
        : Node("receiver")
    {
        m_Sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "pickingPoint/coordinates", 10, std::bind(&Receiver::TopicCallback, this, std::placeholders::_1));
    }

private:
    void TopicCallback(const std_msgs::msg::Int32MultiArray& msg) const
    {
        RCLCPP_INFO(this->get_logger(), "(%d, %d)", msg.data[0], msg.data[1]);
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr m_Sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Receiver>());
    rclcpp::shutdown();
    return 0;
}