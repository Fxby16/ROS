#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

std::string GetProjectRoot()
{
    std::string package_path = ament_index_cpp::get_package_share_directory("ROS_PickingPoint");
    
    int slash_count = 0;

    for(int i = package_path.size() - 1; i >= 0; i--)
    {
        if(package_path[i] == '/')
        {
            slash_count++;
        }

        if(slash_count == 4)
        {
            return package_path.substr(0, i);
        }
    }
    
    return "";
}

class Applicant : public rclcpp::Node
{
public:
    static std::shared_ptr<Applicant> Create()
    {
        auto node = std::make_shared<Applicant>();
        node->Init();
        return node;
    }

    Applicant() : Node("applicant") {}

    void Init()
    {
        // Initialize the image transport publisher
        image_transport::ImageTransport it(shared_from_this());
        m_Pub = it.advertise("camera/image", 10);

        SendImage();
    }

    void SendImage()
    {
        // Load an image using OpenCV
        cv::Mat image = cv::imread(GetProjectRoot() + "/src/pickingPoint/assets/8901.png", cv::IMREAD_COLOR);

        // Check if the image is loaded correctly
        if(image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image!");
            return;
        }

        // Convert OpenCV image to ROS2 message
        std_msgs::msg::Header header;
        header.stamp = this->now();
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

        // Publish the image
        m_Pub.publish(msg);
    }

    image_transport::Publisher m_Pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(Applicant::Create());
    rclcpp::shutdown();
    return 0;
}