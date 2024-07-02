#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <pickingPoint.hpp>

#include <timer.hpp>

class PickingPointHandler : public rclcpp::Node
{
public:
    static std::shared_ptr<PickingPointHandler> Create()
    {
        auto node = std::make_shared<PickingPointHandler>();
        node->Init();
        return node;
    }

    PickingPointHandler() : Node("picking_point_handler") {}

    void Init()
    {
        // Initialize the image transport subscriber
        image_transport::ImageTransport it(shared_from_this());
        m_Pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("pickingPoint/coordinates", 10);
        m_SubMask = it.subscribe("camera/mask_image", 10, std::bind(&PickingPointHandler::ReceiveMask, this, std::placeholders::_1));
        m_SubDepth = it.subscribe("camera/depth_image", 10, std::bind(&PickingPointHandler::ReceiveDepth, this, std::placeholders::_1));

        //printf("Inizializato\n");
    }

    void processImage()
    {
        if (!(m_DepthImage.Loaded && m_MaskImage.Loaded))
            return;

        PickingPoint pp(m_MaskImage.Image, m_DepthImage.Image);
        
        //printf("Oggetto creato\n");
        
        PickingPointInfo pickingPointData = pp.Process();

        //printf("Processato\n");

        std_msgs::msg::Float64MultiArray array;
        // Set up dimensions
        array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        array.layout.dim[0].size = 7;
        array.layout.dim[0].stride = 7;
        array.layout.dim[0].label = "pickingData";
        // Assign the data
        array.data.clear();
        array.data.push_back(static_cast<double>(pickingPointData.point.x));
        array.data.push_back(static_cast<double>(pickingPointData.point.y));
        array.data.push_back(static_cast<double>(pickingPointData.opening[0]));
        array.data.push_back(static_cast<double>(pickingPointData.angle[0]));
        array.data.push_back(static_cast<double>(pickingPointData.opening[1]));
        array.data.push_back(static_cast<double>(pickingPointData.angle[1]));
        array.data.push_back(static_cast<double>(pickingPointData.avgDepth));

        //printf("Creato array\n");

        m_Pub->publish(array);

        //printf("Pubblicato\n");

        m_DepthImage.Unload();
        m_MaskImage.Unload();
    }

    void ReceiveDepth(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "32FC3")->image.clone();

        //printf("Creata depth image\n");

        // Print image information
        RCLCPP_INFO(this->get_logger(), "Image size: %dx%d", image.cols, image.rows);
        RCLCPP_INFO(this->get_logger(), "Image channels: %d", image.channels());

        m_DepthImage.Load(image);

        //printf("Caricata depth image\n");

        processImage();
    }

    void ReceiveMask(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();

        //printf("Creata mask image\n");

        // Print image information
        RCLCPP_INFO(this->get_logger(), "Image size: %dx%d", image.cols, image.rows);
        RCLCPP_INFO(this->get_logger(), "Image channels: %d", image.channels());

        m_MaskImage.Load(image);

        //printf("Caricata mask image\n");

        processImage();
    }

private:

    struct MatWrapped {
        cv::Mat Image;
        bool Loaded;

        MatWrapped() : Image(), Loaded(false) {}

        void Load(cv::Mat& img) {
            Image = img;
            Loaded = true;
        }

        void Unload() {
            Loaded = false;
        }
    };

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_Pub;
    image_transport::Subscriber m_SubMask;
    image_transport::Subscriber m_SubDepth;

    MatWrapped m_DepthImage;
    MatWrapped m_MaskImage;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    cv::namedWindow("view");
    rclcpp::spin(PickingPointHandler::Create());
    cv::destroyWindow("view");
    rclcpp::shutdown();
    return 0;
}