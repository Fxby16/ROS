#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdio>
#include <filesystem>

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
        m_PubFullRGB = it.advertise("camera/full_rgb_image", 10);

        SendImage();
    }

    void SendImage()
    {

        std::string mask_path = "assets/masks";

        std::filesystem::create_directory("output");

        for(const auto& mask_entry : std::filesystem::directory_iterator(mask_path))
        {
            cv::Mat fullrgb = cv::imread(mask_entry.path().string().replace(mask_entry.path().string().find("masks"), 5, "depth") + ".jpg", cv::IMREAD_COLOR);

            std_msgs::msg::Header fullrgb_header;
            fullrgb_header.stamp = this->now();
            sensor_msgs::msg::Image::SharedPtr fullrgb_msg = cv_bridge::CvImage(fullrgb_header, "bgr8", fullrgb).toImageMsg();
            m_PubFullRGB.publish(fullrgb_msg);

            if(fullrgb.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load full rgb image!");
                return;
            }

            for(const auto& entry : std::filesystem::directory_iterator(mask_entry.path()))
            {
                printf("Path: %s\n", entry.path().c_str());

                std::string tmp = entry.path().string().replace(entry.path().string().find("masks"), 5, "depth_masked");
                std::string tmp2 = tmp.replace(tmp.find_last_of("."), 4, ".exr");

                cv::Mat mask = cv::imread(entry.path().string(), cv::IMREAD_COLOR);

                if(mask.empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to load mask image!");
                    return;
                }

                cv::Mat depth = cv::imread(tmp2, cv::IMREAD_UNCHANGED);

                if(depth.empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to load depth image!");
                    return;
                }

                // Print image information
                RCLCPP_INFO(this->get_logger(), "Image size: %dx%d", mask.cols, mask.rows);
                RCLCPP_INFO(this->get_logger(), "Image channels: %d", mask.channels());

                // Print depth image information
                RCLCPP_INFO(this->get_logger(), "Depth image size: %dx%d", depth.cols, depth.rows);
                RCLCPP_INFO(this->get_logger(), "Depth image channels: %d", depth.channels());

                // Convert OpenCV image to ROS2 message
                std_msgs::msg::Header header;
                header.stamp = this->now();
                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", mask).toImageMsg();
                m_PubMask.publish(msg);

                std_msgs::msg::Header depth_header;
                depth_header.stamp = this->now();
                sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(depth_header, "32FC3", depth).toImageMsg();
                m_PubDepth.publish(depth_msg);

                printf("Press any key to continue (q to quit) ...");
                char ch = getchar();
                if(ch == 'q')
                {
                    return;
                }
            }
        }
    }

    image_transport::Publisher m_PubMask;
    image_transport::Publisher m_PubDepth;
    image_transport::Publisher m_PubFullRGB;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(Applicant::Create());
    rclcpp::shutdown();
    return 0;
}