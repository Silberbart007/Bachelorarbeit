#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

// CameraPublisher Klasse zur Veröffentlichung von Kamerabildern
class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher();

private:
  void publish_image();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

// Funktion, um den Kamera-Node zu starten
void start_camera_node();

#endif // CAMERA_NODE_H
