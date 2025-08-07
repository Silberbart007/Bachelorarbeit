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
    painter.setRenderHint(QPainter::Antialiasing);

    if (!m_scan_available)
        return;

    const auto& scan = m_current_scan;

    // Bildabmessungen
    float w = width();
    float h = height();
    float cx = w / 2.0f; // Bildmitte horizontal
    float cy = h / 2.0f; // Bildmitte vertikal

    // Sichtfeld des Lasers (FOV)
    float fov = scan.angle_max - scan.angle_min;

    // "Brennweite" der virtuellen Kamera in Pixel
    float f = w / (2.0f * std::tan(fov / 2.0f));

    size_t start = 25;
    size_t end = scan.ranges.size() - start;

    for (size_t i = start; i < end; ++i) {
        float r = scan.ranges[i];
        if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max)
            continue;

        float angle = scan.angle_min + i * scan.angle_increment;

        // Punkt im Roboter-KS (x = vorne, y = seitlich)
        float x = r * std::cos(angle); // vorwärts
        float y = r * std::sin(angle); // seitlich

        // Perspektivische Projektion: horizontale Position auf dem Bildschirm
        float zoom = 0.9f;
        float u = f * (y / x) * zoom + cx;

        // Perspektivische Höhe: je näher, desto größer
        float bar_height = 400.0f / x; // anpassbar
        bar_height = std::clamp(bar_height, 2.0f, h);

        // Positionen im Bildraum
        float bar_width = 5.0f;
        float x_left = u - bar_width / 2.0f;
        float y_top = cy - bar_height;
        float y_bottom = cy;

        // Farbgebung nach Tiefe
        float norm = std::exp(-x * 0.3f);
        norm = std::clamp(norm, 0.0f, 1.0f);
        float hue = norm * 0.7f;
        QColor color = QColor::fromHsvF(hue, 1.0, 1.0);

        painter.setPen(Qt::NoPen);
        painter.setBrush(color);

        // Balken oben
        y_top = cy - bar_height / 2.0f;
        painter.drawRect(QRectF(x_left, y_top, bar_width, bar_height));
    }
}

/*
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
        float frontal_r = r * std::cos(angle * 1.0f);
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
}*/
