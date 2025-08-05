#include "ego_widget.h"
#include <cmath>

EgoWidget::EgoWidget(QWidget* parent) : QWidget(parent) {
    // setMinimumSize(2000, 500); // Kannst du beliebig anpassen
}

void EgoWidget::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    m_current_scan = *msg;
    m_scan_available = true;
    update(); // Triggert paintEvent
}

void EgoWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    if (!m_scan_available)
        return;

    size_t cut_scan_num = 100;
    const auto& scan = m_current_scan;
    size_t n = scan.ranges.size();
    size_t usable_n = n - 2 * cut_scan_num;

    float w = width();
    float h = height();
    float mid_y = h / 2.0f;
    float column_width = w / static_cast<float>(usable_n);

    for (size_t i = cut_scan_num; i < n - cut_scan_num; ++i) {
        float r = scan.ranges[i];
        if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max)
            continue;

        float angle = scan.angle_min + i * scan.angle_increment;
        float frontal_r = r * std::cos(angle * 0.8f);
        float norm = std::exp(-frontal_r * 0.6f);
        norm = std::clamp(norm, 0.0f, 1.0f);
        float bar_height = norm * (h / 2.0f);

        float x = (usable_n - 1 - (i - cut_scan_num)) * column_width;
        float y_top = mid_y - bar_height;

        float hue = norm * 0.7f;
        QColor color = QColor::fromHsvF(hue, 1.0, 1.0);

        painter.setBrush(color);
        painter.setPen(Qt::NoPen);

        QRectF rect_top(x, y_top, column_width, bar_height);
        painter.drawRect(rect_top);

        QRectF rect_bottom(x, mid_y, column_width, bar_height);
        painter.drawRect(rect_bottom);
    }
}

// void EgoWidget::paintEvent(QPaintEvent* event) {
//     Q_UNUSED(event);
//     QPainter painter(this);
//     painter.fillRect(rect(), Qt::black);

//     if (!m_scan_available)
//         return;

//     const auto& scan = m_current_scan;
//     size_t n = scan.ranges.size();
//     float w = width();
//     float h = height();

//     // Ursprung in der Mitte
//     QPointF origin(w / 2.0, h / 2.0);

//     // Skalierung: z.B. 100 Pixel pro Meter
//     float scale = 100.0f;

//     painter.setPen(Qt::NoPen);

//     for (size_t i = 0; i < n; ++i) {
//         float r = scan.ranges[i];
//         if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max)
//             continue;

//         float angle = scan.angle_min + i * scan.angle_increment;

//         // Kartesische Koordinaten
//         float x = r * std::cos(angle);
//         float y = r * std::sin(angle);

//         // GUI-Koordinaten (y-Achse umgedreht)
//         QPointF point = origin + QPointF(x * scale, -y * scale);

//         // Nähe → Farbe (Rot = nah, Blau = fern)
//         float norm = std::exp(-r * 1.0f);
//         norm = std::clamp(norm, 0.0f, 1.0f);
//         float hue = norm * 0.7f;
//         QColor color = QColor::fromHsvF(hue, 1.0, 1.0);

//         painter.setBrush(color);

//         // Kleiner Punkt
//         painter.drawEllipse(point, 2.0, 2.0);
//     }
// }
