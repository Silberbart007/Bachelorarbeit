#ifndef MAP_SUBSCRIBER_H
#define MAP_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <mutex>

class MapSubscriber : public rclcpp::Node
{
public:
  MapSubscriber();

  // Gibt die zuletzt empfangene Karte zurüc
  nav_msgs::msg::OccupancyGrid get_map();

  // Gibt an, ob überhaupt schon eine Karte empfangen wurde
  bool has_map() const;

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_subscription;
  nav_msgs::msg::OccupancyGrid m_last_map;
  mutable std::mutex m_map_mutex;
  bool m_map_received;
};

#endif  // MAP_SUBSCRIBER_H
