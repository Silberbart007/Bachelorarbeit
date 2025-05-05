#include <QApplication>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RosNode : public rclcpp::Node
{
  public:
    RosNode() : Node("my_noed")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    }

    void publishMessage()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hallo von QT!";
      publisher_->publish(message);
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto ros_node = std::make_shared<RosNode>();

  QPushButton button("Sende Nachricht");
  QObject::connect(&button, &QPushButton::clicked, [&ros_node]() {
    ros_node->publishMessage();
  });

  button.show();

  rclcpp::spin_some(ros_node);
  return app.exec();
}