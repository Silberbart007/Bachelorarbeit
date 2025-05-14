#include "../include/test_qt_pkg/touch_slider_vertical.h"
#include <QPainter>
#include <QDebug>

CustomTouchSliderVertical::CustomTouchSliderVertical(QWidget *parent)
    : QWidget(parent), m_value(0)
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

    const int margin = 10 * dpi;
    const int sliderWidth = 60 * dpi;
    const int sliderHeight = 60 * dpi;

    // Spur zeichnen
    painter.setPen(QPen(Qt::black, 2));
    painter.setBrush(Qt::lightGray);
    painter.drawRect(width() / 2 - sliderWidth / 2, margin, sliderWidth, height() - 2 * margin);

    // Schieberegler zeichnen
    painter.setBrush(Qt::blue);
    int usableHeight = height() - 2 * margin - sliderHeight;

    int sliderY = margin + usableHeight * (100 - m_value) / 100;
    painter.drawRect(width() / 2 - sliderWidth / 2 - 5, sliderY, sliderWidth + 10, sliderHeight);
}



bool CustomTouchSliderVertical::event(QEvent *event)
{
    if (event->type() == QEvent::TouchBegin || event->type() == QEvent::TouchUpdate || event->type() == QEvent::TouchEnd) {
        QTouchEvent *touchEvent = static_cast<QTouchEvent *>(event);
        QList<QTouchEvent::TouchPoint> touchPoints = touchEvent->touchPoints();

        for (const QTouchEvent::TouchPoint &point : touchPoints) {
            QPointF touchPos = point.pos();  // Position des Touchpoints
            if (rect().contains(touchPos.toPoint())) {  // Überprüfen, ob der Touchpunkt im Widget-Bereich liegt
                int newValue = std::clamp(mapToSliderValue(touchPos.y()), 0, 100);  // Berechnet den neuen Wert basierend auf der X-Position
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
        int newValue = mapToSliderValue(event->y());  // Berechnet den neuen Wert basierend auf der X-Position
        setValue(newValue);
    }
}

void CustomTouchSliderVertical::mouseMoveEvent(QMouseEvent *event)
{
    if (rect().contains(event->pos())) {  // Überprüfen, ob der Mauszeiger im Widget-Bereich ist
        int newValue = mapToSliderValue(event->y());  // Berechnet den neuen Wert basierend auf der X-Position
        setValue(newValue);
        //qDebug() << "Value: " << newValue;
    }
}

void CustomTouchSliderVertical::mouseReleaseEvent(QMouseEvent *)
{
    // Optional: Hier kann man spezielle Logik nach dem Loslassen der Maus hinzufügen, falls gewünscht.
}

int CustomTouchSliderVertical::mapToSliderValue(int y)
{
    const int margin = 10;
    const int sliderHeight = 30;
    int usableHeight = height() - 2 * margin - sliderHeight;

y = std::clamp(y, margin + sliderHeight / 2, margin + usableHeight + sliderHeight / 2);
    int relativeY = y - margin;

    int value = 100 - ((relativeY - sliderHeight/2)* 100 / usableHeight);
    return std::clamp(value, 0, 100);
}



void CustomTouchSliderVertical::setValue(int newValue)
{
    if (newValue != m_value) {
        m_value = newValue;

        // An Roboter publishen
        m_robot_node->publish_velocity(m_value, m_robot_node->getRotation());

        update();  // Widget neu zeichnen, um die Slider-Position zu aktualisieren
    }
}
