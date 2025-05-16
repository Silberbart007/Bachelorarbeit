// map_simulator.h

#pragma once

#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>

struct SimulatedObstacle {
  float x;
  float y;
  float radius; // Kreisförmiges Hindernis
};

struct SimulatedRobot {
  float x = 2.0f;   // Position (m)
  float y = 2.0f;
  float theta = 0.0f; // Orientierung (rad)
};

class MapSimulator : public rclcpp::Node {
public:
  MapSimulator();

private:
  void timer_callback();
  sensor_msgs::msg::LaserScan generate_laser_scan();

  SimulatedRobot robot_;
  std::vector<SimulatedObstacle> obstacles_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
};


void start_map_simulator_node();
