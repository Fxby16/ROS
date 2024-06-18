#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <std_msgs/msg/int32_multi_array.hpp>

#include <pickingPoint.hpp>

class ImageSubscriberNode : public rclcpp::Node
{
public:
    static std::shared_ptr<ImageSubscriberNode> create()
    {
        auto node = std::make_shared<ImageSubscriberNode>();
        node->init();
        return node;
    }

    ImageSubscriberNode() : Node("image_subscriber") {}

    void init()
    {
        // Initialize the image transport subscriber
		printf("Init..\n");
        image_transport::ImageTransport it(shared_from_this());
		pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("pickingPoint/coordinates", 10);
        sub_ = it.subscribe("camera/image", 10, std::bind(&ImageSubscriberNode::image_callback, this, std::placeholders::_1));
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
		printf("Callback..\n");
        try
        {
            // Convert ROS2 message to OpenCV image
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

            PickingPoint pp(image);

            cv::Point pickingPoint = pp.Process();

            // Create a Int32MultiArray message
			std_msgs::msg::Int32MultiArray array;
			// Set up dimensions
			array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
			array.layout.dim[0].size = 2;
			array.layout.dim[0].stride = 2;
			array.layout.dim[0].label = "x_y_coordinates"; // or whatever name you typically use to refer to a point
			// Assign the data
			array.data.clear();
			array.data.push_back(pickingPoint.x);
			array.data.push_back(pickingPoint.y);

			printf("Picking point: (%d, %d)\n", pickingPoint.x, pickingPoint.y);
            pub_->publish(array);
        }
        catch (cv_bridge::Exception &e)
        {	
			printf("Error..\n");
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    image_transport::Subscriber sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    cv::namedWindow("view");
    rclcpp::spin(ImageSubscriberNode::create());
    cv::destroyWindow("view");
    rclcpp::shutdown();
    return 0;
}