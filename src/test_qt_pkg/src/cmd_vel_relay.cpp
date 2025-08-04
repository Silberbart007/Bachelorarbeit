#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelRelayNode : public rclcpp::Node
{
public:
  CmdVelRelayNode()
  : Node("cmd_vel_relay_node")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_smoothed", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        pub_->publish(*msg);
      });
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelRelayNode>());
  rclcpp::shutdown();
  return 0;
}
