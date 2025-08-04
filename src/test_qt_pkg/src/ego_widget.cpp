#include "ego_widget.h"
#include <cmath>

EgoWidget::EgoWidget(QWidget* parent) : QWidget(parent) {
    //setMinimumSize(2000, 500); // Kannst du beliebig anpassen
}

void EgoWidget::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    m_current_scan = *msg;
    m_scan_available = true;
    update(); // Triggert paintEvent
}

void EgoWidget::paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    if (!m_scan_available)
        return;

    const auto& scan = m_current_scan;
    size_t n = scan.ranges.size();
    float w = width();
    float h = height();
    float mid_y = h / 2.0f;
    float column_width = w / static_cast<float>(n);

    for (size_t i = 0; i < n; ++i) {
        float r = scan.ranges[i];
        if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max)
            continue;

        // Stärke betonen: Näher = größer
        float norm = std::exp(-r * 2.0f);
        norm = std::clamp(norm, 0.0f, 1.0f);
        float bar_height = norm * (h / 2.0f);

        float x = (n - 1 - i) * column_width;
        float y_top = mid_y - bar_height;

        // Farbe: hue von rot (nah) → blau (fern)
        float hue = norm * 0.7f; // 0.0 = rot, 0.7 = blau
        QColor color = QColor::fromHsvF(hue, 1.0, 1.0);

        painter.setBrush(color);
        painter.setPen(Qt::NoPen);

        // Nach oben zeichnen
        QRectF rect_top(x, y_top, column_width, bar_height);
        painter.drawRect(rect_top);

        // Nach unten spiegeln
        QRectF rect_bottom(x, mid_y, column_width, bar_height);
        painter.drawRect(rect_bottom);
    }
}
