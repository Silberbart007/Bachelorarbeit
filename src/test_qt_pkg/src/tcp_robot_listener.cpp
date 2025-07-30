#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>

#include "bridge_protocol.h"  // enthält CombinedData, LaserScanData, TFData

class TcpRobotListenerNode : public rclcpp::Node {
public:
    TcpRobotListenerNode()
        : Node("tcp_robot_listener")
    {
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

        socket_fd_ = connect_to_server("172.26.1.1", 2077);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Verbindung zum Server fehlgeschlagen");
            rclcpp::shutdown();
            return;
        }

        recv_thread_ = std::thread(&TcpRobotListenerNode::receive_loop, this);
        recv_thread_.detach();
    }

    ~TcpRobotListenerNode() {
        if (socket_fd_ >= 0) close(socket_fd_);
    }

private:
    int socket_fd_;
    std::thread recv_thread_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

    int connect_to_server(const std::string& ip, int port) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) return -1;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);

        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
            return -1;

        RCLCPP_INFO(this->get_logger(), "Verbindung hergestellt zu %s:%d", ip.c_str(), port);
        return sock;
    }

    void receive_loop() {
        while (rclcpp::ok()) {
            CombinedData data;
            if (!recv_all((char*)&data, sizeof(data))) {
                RCLCPP_WARN(this->get_logger(), "Fehler beim Empfang von CombinedData");
                break;
            }

            publish_laser(data.laser_data);
            publish_tf(data.tf_data);
        }
    }

    bool recv_all(char* buffer, size_t size) {
        size_t received = 0;
        while (received < size) {
            ssize_t n = recv(socket_fd_, buffer + received, size - received, 0);
            if (n <= 0) return false;
            received += n;
        }
        return true;
    }

    void publish_laser(const LaserScanData& d) {
        auto msg = sensor_msgs::msg::LaserScan();
        msg.header.stamp = rclcpp::Time(d.stamp * 1e9);
        msg.header.frame_id = std::string(d.frame_id);
        msg.angle_min = d.angle_min;
        msg.angle_max = d.angle_max;
        msg.angle_increment = d.angle_increment;
        msg.time_increment = d.time_increment;
        msg.scan_time = d.scan_time;
        msg.range_min = d.range_min;
        msg.range_max = d.range_max;
        msg.ranges.assign(d.ranges, d.ranges + 541);
        msg.intensities.assign(d.intensities, d.intensities + 541);

        laser_pub_->publish(msg);
    }

    void publish_tf(const TFData& d) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = rclcpp::Time(d.stamp * 1e9);
        tf.header.frame_id = std::string(d.frame_id);
        tf.child_frame_id = std::string(d.child_frame_id);
        tf.transform.translation.x = d.trans[0];
        tf.transform.translation.y = d.trans[1];
        tf.transform.translation.z = d.trans[2];
        tf.transform.rotation.x = d.rot[0];
        tf.transform.rotation.y = d.rot[1];
        tf.transform.rotation.z = d.rot[2];
        tf.transform.rotation.w = d.rot[3];

        tf2_msgs::msg::TFMessage tf_msg;
        tf_msg.transforms.push_back(tf);
        tf_pub_->publish(tf_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpRobotListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
