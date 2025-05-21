#include "map_subscriber.h"

MapSubscriber::MapSubscriber()
: Node("map_subscriber"), m_map_received(false)
{
  m_map_subscription = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&MapSubscriber::map_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "MapSubscriber Node started. Waiting for /map...");
}

void MapSubscriber::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(m_map_mutex);
  m_last_map = *msg;
  m_map_received = true;
  RCLCPP_INFO(this->get_logger(), "Map received: %d x %d", msg->info.width, msg->info.height);
}

nav_msgs::msg::OccupancyGrid MapSubscriber::get_map()
{
  std::lock_guard<std::mutex> lock(m_map_mutex);
  return m_last_map;
}

bool MapSubscriber::has_map() const
{
  return m_map_received;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapSubscriber>();  // oder dein Node-Name
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
