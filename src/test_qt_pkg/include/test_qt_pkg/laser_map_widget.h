#pragma once

#include <QPainter>
#include <QWidget>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserMapWidget : public QWidget {
    Q_OBJECT

  public:
    explicit LaserMapWidget(QWidget* parent = nullptr);

    // Callback für ROS
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  protected:
    void paintEvent(QPaintEvent* event) override;

  private:
    sensor_msgs::msg::LaserScan m_current_scan;
    bool m_scan_available = false;
};
