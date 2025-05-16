// map_simulator.cpp

#include "../include/test_qt_pkg/map_simulator.h"
#include <cmath>

MapSimulator::MapSimulator() : Node("map_simulator") {
  laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MapSimulator::timer_callback, this));

  // Hindernis hinzufügen
  obstacles_.push_back({4.0f, 2.0f, 0.3f}); // Kreisförmiges Hindernis
}

void MapSimulator::timer_callback() {
  auto scan = generate_laser_scan();
  laser_pub_->publish(scan);
}

sensor_msgs::msg::LaserScan MapSimulator::generate_laser_scan() {
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = this->now();
  scan.header.frame_id = "base_link";
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 180.0; // 1° Auflösung
  scan.range_min = 0.1f;
  scan.range_max = 5.0f;

  int num_rays = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
  scan.ranges.resize(num_rays, scan.range_max);

  for (int i = 0; i < num_rays; ++i) {
    float angle = scan.angle_min + i * scan.angle_increment;
    float best_range = scan.range_max;

    for (float r = 0.1f; r < scan.range_max; r += 0.05f) {
      float test_x = robot_.x + r * std::cos(angle + robot_.theta);
      float test_y = robot_.y + r * std::sin(angle + robot_.theta);

      for (const auto& obs : obstacles_) {
        float dx = test_x - obs.x;
        float dy = test_y - obs.y;
        if (std::sqrt(dx * dx + dy * dy) < obs.radius) {
          best_range = r;
          goto found;
        }
      }
    }
  found:
    scan.ranges[i] = best_range;
  }

  return scan;
}

void start_map_simulator_node() {
  rclcpp::spin(std::make_shared<MapSimulator>());
}
