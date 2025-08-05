#include "laser_map_widget.h"
#include <cmath>

LaserMapWidget::LaserMapWidget(QWidget* parent) : QWidget(parent) {}

void LaserMapWidget::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    m_current_scan = *msg;
    m_scan_available = true;
    update(); // Triggert paintEvent
}

void LaserMapWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    if (!m_scan_available)
        return;

    const auto& scan = m_current_scan;
    size_t n = scan.ranges.size();
    float w = width();
    float h = height();

    // Ursprung in der Mitte
    QPointF origin(w / 2.0, h / 2.0);

    // Skalierung: z.B. 100 Pixel pro Meter
    float scale = 100.0f;

    painter.setPen(Qt::NoPen);

    for (size_t i = 0; i < n; ++i) {
        float r = scan.ranges[i];
        if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max)
            continue;

        float angle = scan.angle_min + i * scan.angle_increment + M_PI_2;
        
        // Kartesische Koordinaten
        float x = r * std::cos(angle);
        float y = r * std::sin(angle);

        // GUI-Koordinaten (y-Achse umgedreht)
        QPointF point = origin + QPointF(x * scale, -y * scale);

        // Nähe → Farbe (Rot = nah, Blau = fern)
        float norm = std::exp(-r * 1.0f);
        norm = std::clamp(norm, 0.0f, 1.0f);
        float hue = norm * 0.7f;
        QColor color = QColor::fromHsvF(hue, 1.0, 1.0);

        painter.setBrush(color);

        // Kleiner Punkt
        painter.drawEllipse(point, 4.0, 4.0);
    }

    // Draw Fov Barriers
    painter.setPen(QPen(Qt::white, 2.0, Qt::SolidLine));

    float fov_length = scan.range_max;

    // angle_min
    {
        float angle = scan.angle_min + M_PI_2;
        float x = fov_length * std::cos(angle);
        float y = fov_length * std::sin(angle);
        QPointF end_point = origin + QPointF(x * scale, -y * scale);
        painter.drawLine(origin, end_point);
    }

    // angle_max
    {
        float angle = scan.angle_max + M_PI_2;
        float x = fov_length * std::cos(angle);
        float y = fov_length * std::sin(angle);
        QPointF end_point = origin + QPointF(x * scale, -y * scale);
        painter.drawLine(origin, end_point);
    }

    // Draw Robot
    painter.setBrush(QColor(Qt::white));
    QPointF origin_offset(origin.x(), origin.y() + 15);
    painter.drawEllipse(origin_offset, 16.0, 16.0);
}