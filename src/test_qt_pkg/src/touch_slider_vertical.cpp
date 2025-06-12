#include "touch_slider_vertical.h"
#include <QPainter>
#include <QDebug>

CustomTouchSliderVertical::CustomTouchSliderVertical(QWidget *parent)
    : QWidget(parent), m_value(0.0)
{
    setAttribute(Qt::WA_AcceptTouchEvents, true);  // Touch-Ereignisse akzeptieren
}

int CustomTouchSliderVertical::getValue() const
{
    return m_value;  // Gibt den aktuellen Sliderwert zurück
}

void CustomTouchSliderVertical::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    qreal dpi = this->devicePixelRatioF();

    const double margin = 10 * dpi;
    const double sliderWidth = 60 * dpi;
    const double sliderHeight = 60 * dpi;

    // Spur zeichnen
    painter.setPen(QPen(Qt::black, 2));
    painter.setBrush(Qt::lightGray);
    painter.drawRect(width() / 2 - sliderWidth / 2, margin, sliderWidth, height() - 2 * margin);

    // Schieberegler zeichnen
    painter.setBrush(Qt::blue);
    double usableHeight = height() - 2 * margin - sliderHeight;
    
    // Normalisiere m_value
    double normValue = (m_value + 1.0) / 2.0;

    double sliderY = margin + usableHeight * (1.0 - normValue);  // oben = 1.0

    painter.drawRect(width() / 2 - sliderWidth / 2 - 5, sliderY, sliderWidth + 10, sliderHeight);

    // Deadzone grafisch darstellen
    // Deadzone von ±0.1 anzeigen
    double deadzone_top = sliderValueToPixels(0.1) + sliderHeight / 2.0;
    double deadzone_bottom = sliderValueToPixels(-0.1) + sliderHeight / 2.0;
    double deadzone_height = deadzone_bottom - deadzone_top;

    painter.setBrush(QColor(200, 0, 0, 120));  // halbtransparentes Rot
    painter.setPen(Qt::NoPen);
    painter.drawRect(width() / 2 - sliderWidth / 2,
                    deadzone_top,
                    sliderWidth,
                    deadzone_height);
}



bool CustomTouchSliderVertical::event(QEvent *event)
{
    if (event->type() == QEvent::TouchBegin || event->type() == QEvent::TouchUpdate || event->type() == QEvent::TouchEnd) {
        QTouchEvent *touchEvent = static_cast<QTouchEvent *>(event);
        QList<QTouchEvent::TouchPoint> touchPoints = touchEvent->touchPoints();

        for (const QTouchEvent::TouchPoint &point : touchPoints) {
            QPointF touchPos = point.pos();  // Position des Touchpoints
            if (rect().contains(touchPos.toPoint())) {  // Überprüfen, ob der Touchpunkt im Widget-Bereich liegt
                double newValue = std::clamp(mapToSliderValue(touchPos.y()), -1.0, 1.0);  // Berechnet den neuen Wert basierend auf der X-Position
                setValue(newValue);
            }
        }
        return true;  // Touch-Ereignis wurde verarbeitet
    }
    return QWidget::event(event);  // Anderen Ereignissen Standardverhalten geben
}

void CustomTouchSliderVertical::mousePressEvent(QMouseEvent *event)
{
    if (rect().contains(event->pos())) {  // Überprüfen, ob der Mauszeiger im Widget-Bereich ist
        double newValue = mapToSliderValue(event->y());  // Berechnet den neuen Wert basierend auf der X-Position
        setValue(newValue);
    }
}

void CustomTouchSliderVertical::mouseMoveEvent(QMouseEvent *event)
{
    if (rect().contains(event->pos())) {  // Überprüfen, ob der Mauszeiger im Widget-Bereich ist
        double newValue = mapToSliderValue(event->y());  // Berechnet den neuen Wert basierend auf der X-Position
        setValue(newValue);
        //qDebug() << "Value: " << newValue;
    }
}

void CustomTouchSliderVertical::mouseReleaseEvent(QMouseEvent *)
{
    // Optional: Hier kann man spezielle Logik nach dem Loslassen der Maus hinzufügen, falls gewünscht.
}

double CustomTouchSliderVertical::mapToSliderValue(double y)
{
    const double margin = 10;
    const double sliderHeight = 60;
    int usableHeight = height() - 2 * margin - sliderHeight;

    y = std::clamp(y, margin + sliderHeight / 2, margin + usableHeight + sliderHeight / 2);
    double relativeY = y - margin - sliderHeight / 2;

    // 0.0 → unten, 1.0 → oben
    double normValue = 1.0 - (relativeY / usableHeight);

    // Mapp auf [-1.0, 1.0]
    return std::clamp(normValue * 2.0 - 1.0, -1.0, 1.0);
}




void CustomTouchSliderVertical::setValue(double newValue)
{
    if (newValue != m_value) {
        m_value = newValue;

        if (std::abs(newValue) < 0.1)
            m_value = 0.0;

        // Roboterdaten kriegen
        RobotNode::RobotSpeed currentSpeed = m_robot_node->getSpeedNormalized();
        double currentRot = m_robot_node->getRotationNormalized();

        // Korrekte Daten publishen
        currentSpeed.x = m_value;
        m_robot_node->publish_velocity(currentSpeed, currentRot);

        update();  // Widget neu zeichnen, um die Slider-Position zu aktualisieren
    }
}

double CustomTouchSliderVertical::sliderValueToPixels(double value) const
{
    qreal dpi = this->devicePixelRatioF();
    const double margin = 10 * dpi;
    const double sliderHeight = 60 * dpi;
    double usableHeight = height() - 2 * margin - sliderHeight;

    double normValue = (value + 1.0) / 2.0;

    // Wert 1.0 (oben) → kleiner Y
    return margin + usableHeight * (1.0 - normValue);
}
