#pragma once

#include <QDebug>
#include <QPainter>
#include <QWidget>
#include <QtMath>
#include <sensor_msgs/msg/laser_scan.hpp>

class EgoWidget : public QWidget {
    Q_OBJECT

  public:
    explicit EgoWidget(QWidget* parent = nullptr);

    // Wird von außen aufgerufen (z.B. in RobotNode)
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // setter
    void setZoomFactor(double zoom_factor) {
        m_zoom_factor = zoom_factor;
    };

    // Getter
    double getZoomFactor() {
        return m_zoom_factor;
    };

  protected:
    void paintEvent(QPaintEvent* event) override;

  private:
    sensor_msgs::msg::LaserScan m_current_scan;
    bool m_scan_available = false;
    double m_zoom_factor;
};
