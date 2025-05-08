#include "../include/test_qt_pkg/cam.h"

CameraPublisher::CameraPublisher() : Node("camera_publisher") {
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&CameraPublisher::publish_image, this));
  cap_.open(0, cv::CAP_V4L2);  // USB-Kamera
}

void CameraPublisher::publish_image() {
  cv::Mat frame;
  cap_ >> frame;
  if (!frame.empty()) {
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    publisher_->publish(*msg);
  }
}

void start_camera_node() {
  rclcpp::spin(std::make_shared<CameraPublisher>());
}
