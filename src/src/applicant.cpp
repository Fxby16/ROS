#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImagePublisherNode : public rclcpp::Node
{
public:
    static std::shared_ptr<ImagePublisherNode> create()
    {
        auto node = std::make_shared<ImagePublisherNode>();
        node->init();
        return node;
    }

    ImagePublisherNode() : Node("image_publisher") {}

    void init()
    {
        // Initialize the image transport publisher
        image_transport::ImageTransport it(shared_from_this());
        pub_ = it.advertise("camera/image", 10);

        // Create a timer to periodically publish images
        /*timer_ = this->create_wall_timer(
          std::chrono::milliseconds(100), std::bind(&ImagePublisherNode::timer_callback, this));
        */

        sendImage();
    }

    void sendImage()
    {
        // Load an image using OpenCV (replace this with your actual image loading code)
        cv::Mat image = cv::imread("/mnt/574cdeb8-9b3d-49a4-aa16-95589af03bdf/ROS/src/pickingPoint/assets/8901.png", cv::IMREAD_COLOR);

        // Check if the image is loaded correctly
        if (image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image!");
            return;
        }

        // Convert OpenCV image to ROS2 message
        std_msgs::msg::Header header;
        header.stamp = this->now();
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

        // Publish the image
        pub_.publish(msg);
    }

    image_transport::Publisher pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(ImagePublisherNode::create());
    rclcpp::shutdown();
    return 0;
}