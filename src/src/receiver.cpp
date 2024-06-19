#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>

class Receiver : public rclcpp::Node
{
public:
    Receiver()
        : Node("receiver")
    {
        m_Sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "pickingPoint/coordinates", 10, std::bind(&Receiver::TopicCallback, this, std::placeholders::_1));
    }

private:
    void TopicCallback(const std_msgs::msg::Float64MultiArray& msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Position: (%.2lf, %.2lf)", msg.data[0], msg.data[1]);
        RCLCPP_INFO(this->get_logger(), "1) Opening %.2lf, Angle %.2lf", msg.data[2], msg.data[3]);
        RCLCPP_INFO(this->get_logger(), "2) Opening %.2lf, Angle %.2lf", msg.data[4], msg.data[5]);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_Sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Receiver>());
    rclcpp::shutdown();
    return 0;
}