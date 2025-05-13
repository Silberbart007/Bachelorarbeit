#include "../include/test_qt_pkg/touch_slider.h"
#include <QPainter>
#include <QDebug>

CustomTouchSlider::CustomTouchSlider(QWidget *parent)
    : QWidget(parent), m_value(0)
{
    setAttribute(Qt::WA_AcceptTouchEvents, true);  // Touch-Ereignisse akzeptieren
}

int CustomTouchSlider::getValue() const
{
    return m_value;  // Gibt den aktuellen Sliderwert zurück
}

void CustomTouchSlider::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Hintergrund zeichnen (Slider-Spur)
    painter.setPen(QPen(Qt::black, 2));
    painter.setBrush(Qt::lightGray);
    painter.drawRect(width() / 2 - 10, 10, 20, height() - 20);

    // Schieberegler zeichnen
    painter.setBrush(Qt::blue);

    int sliderHeight = 30;
    int sliderY = height() - 10 - ((m_value * (height() - 20 - sliderHeight)) / 100);

    painter.drawRect(width() / 2 - 15, sliderY, 30, sliderHeight);
}


bool CustomTouchSlider::event(QEvent *event)
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

void CustomTouchSlider::mousePressEvent(QMouseEvent *event)
{
    if (rect().contains(event->pos())) {  // Überprüfen, ob der Mauszeiger im Widget-Bereich ist
        int newValue = mapToSliderValue(event->x());  // Berechnet den neuen Wert basierend auf der X-Position
        setValue(newValue);
    }
}

void CustomTouchSlider::mouseMoveEvent(QMouseEvent *event)
{
    if (rect().contains(event->pos())) {  // Überprüfen, ob der Mauszeiger im Widget-Bereich ist
        int newValue = mapToSliderValue(event->x());  // Berechnet den neuen Wert basierend auf der X-Position
        setValue(newValue);
    }
}

void CustomTouchSlider::mouseReleaseEvent(QMouseEvent *)
{
    // Optional: Hier kann man spezielle Logik nach dem Loslassen der Maus hinzufügen, falls gewünscht.
}

int CustomTouchSlider::mapToSliderValue(int y)
{
    int usableHeight = height() - 20 - 30;  // Abzug: Ränder und Slider-Höhe
    int relativeY = std::clamp(y - 10, 0, usableHeight);
    int value = 100 - (relativeY * 100 / usableHeight);
    return std::clamp(value, 0, 100);
}

void CustomTouchSlider::setValue(int newValue)
{
    if (newValue != m_value) {
        m_value = newValue;
        update();  // Widget neu zeichnen, um die Slider-Position zu aktualisieren
    }
}
