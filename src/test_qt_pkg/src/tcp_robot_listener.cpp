#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <cstring>

class TcpRobotListener : public rclcpp::Node {
public:
    TcpRobotListener()
    : Node("tcp_robot_listener"), socket_fd_(-1)
    {
        this->declare_parameter<std::string>("robot_ip", "172.26.1.1");
        this->declare_parameter<int>("robot_port", 2077);
        this->get_parameter("robot_ip", robot_ip_);
        this->get_parameter("robot_port", robot_port_);

        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/base_scan", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        connect_to_robot();
        recv_thread_ = std::thread(&TcpRobotListener::recv_loop, this);
    }

    ~TcpRobotListener() {
        running_ = false;
        if (recv_thread_.joinable()) recv_thread_.join();
        if (socket_fd_ != -1) close(socket_fd_);
    }

private:
    void connect_to_robot() {
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return;
        }

        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(robot_port_);
        inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr);

        if (connect(socket_fd_, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection to robot failed");
            close(socket_fd_);
            socket_fd_ = -1;
        } else {
            RCLCPP_INFO(this->get_logger(), "Connected to robot at %s:%d", robot_ip_.c_str(), robot_port_);
        }
    }

    void recv_loop() {
        running_ = true;
        while (running_ && socket_fd_ != -1) {
            // Header: 1 byte type + 4 byte length
            uint8_t msg_type;
            uint32_t payload_size = 0;

            if (!recv_all((char*)&msg_type, 1)) break;
            if (!recv_all((char*)&payload_size, 4)) break;

            std::vector<char> payload(payload_size);
            if (!recv_all(payload.data(), payload_size)) break;

            if (msg_type == 0x01) {
                handle_tf(payload);
            } else if (msg_type == 0x02) {
                handle_laserscan(payload);
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown message type: %d", msg_type);
            }
        }

        RCLCPP_WARN(this->get_logger(), "Disconnected from robot");
    }

    void handle_tf(const std::vector<char>& data) {
        const char* ptr = data.data();

        float tx = *(float*)(ptr); ptr += 4;
        float ty = *(float*)(ptr); ptr += 4;
        float tz = *(float*)(ptr); ptr += 4;

        float qx = *(float*)(ptr); ptr += 4;
        float qy = *(float*)(ptr); ptr += 4;
        float qz = *(float*)(ptr); ptr += 4;
        float qw = *(float*)(ptr); ptr += 4;

        std::string frame_id(ptr, 32); ptr += 32;
        std::string child_frame_id(ptr, 32); ptr += 32;

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = frame_id.c_str();
        tf.child_frame_id = child_frame_id.c_str();

        tf.transform.translation.x = tx;
        tf.transform.translation.y = ty;
        tf.transform.translation.z = tz;
        tf.transform.rotation.x = qx;
        tf.transform.rotation.y = qy;
        tf.transform.rotation.z = qz;
        tf.transform.rotation.w = qw;

        tf_broadcaster_->sendTransform(tf);
    }

    void handle_laserscan(const std::vector<char>& data) {
        const char* ptr = data.data();

        float angle_min = *(float*)(ptr); ptr += 4;
        float angle_increment = *(float*)(ptr); ptr += 4;
        float range_min = *(float*)(ptr); ptr += 4;
        float range_max = *(float*)(ptr); ptr += 4;

        uint32_t count = *(uint32_t*)(ptr); ptr += 4;

        std::vector<float> ranges(count);
        std::memcpy(ranges.data(), ptr, count * sizeof(float));

        sensor_msgs::msg::LaserScan msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "laser";
        msg.angle_min = angle_min;
        msg.angle_increment = angle_increment;
        msg.angle_max = angle_min + angle_increment * count;
        msg.range_min = range_min;
        msg.range_max = range_max;
        msg.ranges = ranges;

        laser_pub_->publish(msg);
    }

    // returns false if connection broke
    bool recv_all(char* buffer, size_t size) {
        size_t total = 0;
        while (total < size) {
            ssize_t n = recv(socket_fd_, buffer + total, size - total, MSG_WAITALL);
            if (n <= 0) return false;
            total += n;
        }
        return true;
    }

    std::string robot_ip_;
    int robot_port_;
    int socket_fd_;
    std::atomic<bool> running_;

    std::thread recv_thread_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpRobotListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
