#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdio>

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
        m_PubMask = it.advertise("camera/mask_image", 10);
        m_PubDepth = it.advertise("camera/depth_image", 10);

        SendImage();
    }

    void SendImage()
    {
        // Load an image using OpenCV
        //cv::Mat image = cv::imread(GetProjectRoot() + "/src/pickingPoint/assets/bin_mask_2.png", cv::IMREAD_COLOR);
        
        cv::Mat image = cv::imread("/mnt/574cdeb8-9b3d-49a4-aa16-95589af03bdf/ROS/src/pickingPoint/assets/bin_mask_2.png", cv::IMREAD_COLOR);

        if(image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image!");
            return;
        }
        
        // cv::Mat depth = cv::imread(GetProjectRoot() + "/src/pickingPoint/assets/bin_mask_2.exr", cv::IMREAD_UNCHANGED);

        cv::Mat depth = cv::imread("/mnt/574cdeb8-9b3d-49a4-aa16-95589af03bdf/ROS/src/pickingPoint/assets/bin_mask_2.exr", cv::IMREAD_UNCHANGED);

        if(depth.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load depth image!");
            return;
        }

        // Print image information
        RCLCPP_INFO(this->get_logger(), "Image size: %dx%d", image.cols, image.rows);
        RCLCPP_INFO(this->get_logger(), "Image channels: %d", image.channels());

        // Print depth image information
        RCLCPP_INFO(this->get_logger(), "Depth image size: %dx%d", depth.cols, depth.rows);
        RCLCPP_INFO(this->get_logger(), "Depth image channels: %d", depth.channels());

        //cv::imshow("Image", image);
        //cv::waitKey(0);
        //cv::destroyWindow("Image");
        //cv::imshow("Depth", depth);
        //cv::waitKey(0);
        //cv::destroyWindow("Depth");

        // Convert OpenCV image to ROS2 message
        std_msgs::msg::Header header;
        header.stamp = this->now();
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

        // Publish the mask
        m_PubMask.publish(msg);

        std_msgs::msg::Header depth_header;
        depth_header.stamp = this->now();

        if(depth.type() != CV_32FC3)
        {
            RCLCPP_ERROR(this->get_logger(), "Depth image is not in CV_32FC3 format!");
            return;
        }

        sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(depth_header, "32FC3", depth).toImageMsg();
    
        // Publish the depth
        m_PubDepth.publish(depth_msg);
    }

    image_transport::Publisher m_PubMask;
    image_transport::Publisher m_PubDepth;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(Applicant::Create());
    rclcpp::shutdown();
    return 0;
}