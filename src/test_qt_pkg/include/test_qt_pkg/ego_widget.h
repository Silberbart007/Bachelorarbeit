#pragma once

#include <QWidget>
#include <QPainter>
#include <QtMath>
#include <QDebug>
#include <sensor_msgs/msg/laser_scan.hpp>

class EgoWidget : public QWidget
{
    Q_OBJECT

public:
    explicit EgoWidget(QWidget* parent = nullptr);

    // Wird von außen aufgerufen (z.B. in RobotNode)
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    sensor_msgs::msg::LaserScan m_current_scan;
    bool m_scan_available = false;
};
