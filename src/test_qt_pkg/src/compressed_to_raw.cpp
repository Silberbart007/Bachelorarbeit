#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

class CompressedToRawNode : public rclcpp::Node {
public:
    CompressedToRawNode() : Node("compressed_to_raw") {
        using std::placeholders::_1;

        sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera/image_raw/compressed", 10,
            std::bind(&CompressedToRawNode::callback, this, _1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10);

        RCLCPP_INFO(this->get_logger(), "Node gestartet: Konvertiert compressed → raw");
    }

private:
    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            // Dekomprimieren mit OpenCV
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (image.empty()) {
                RCLCPP_WARN(this->get_logger(), "Leeres Bild nach imdecode.");
                return;
            }

            // OpenCV → ROS Image
            std_msgs::msg::Header header;
            header.stamp = msg->header.stamp;
            header.frame_id = msg->header.frame_id;

            sensor_msgs::msg::Image::SharedPtr ros_image =
                cv_bridge::CvImage(header, "rgb8", image).toImageMsg();

            pub_->publish(*ros_image);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Dekomprimieren: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompressedToRawNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}