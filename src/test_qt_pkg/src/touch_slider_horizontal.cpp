#include "../include/test_qt_pkg/touch_slider_horizontal.h"
#include <QPainter>
#include <QDebug>

CustomTouchSliderHorizontal::CustomTouchSliderHorizontal(QWidget *parent)
    : QWidget(parent), m_value(0)
{
    setAttribute(Qt::WA_AcceptTouchEvents, true);  // Touch-Ereignisse akzeptieren
}

int CustomTouchSliderHorizontal::getValue() const
{
    return m_value;  // Gibt den aktuellen Sliderwert zurück
}

void CustomTouchSliderHorizontal::paintEvent(QPaintEvent *)
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
    painter.drawRect(margin, height() / 2 - sliderHeight / 2, width() - 2 * margin, sliderHeight);

    // Schieberegler zeichnen (Mittelpunkt auf Position!)
    painter.setBrush(Qt::blue);
    int usableWidth = width() - 2 * margin - sliderWidth;
    int centerX = margin + (usableWidth * m_value / 100) + sliderWidth / 2;

    // Ziehe die Hälfte der Sliderbreite ab, um zentriert zu zeichnen
    painter.drawRect(centerX - sliderWidth / 2, height() / 2 - sliderHeight / 2, sliderWidth, sliderHeight);
}


bool CustomTouchSliderHorizontal::event(QEvent *event)
{
    if (event->type() == QEvent::TouchBegin || event->type() == QEvent::TouchUpdate || event->type() == QEvent::TouchEnd) {
        QTouchEvent *touchEvent = static_cast<QTouchEvent *>(event);
        QList<QTouchEvent::TouchPoint> touchPoints = touchEvent->touchPoints();

        for (const QTouchEvent::TouchPoint &point : touchPoints) {
            QPointF touchPos = point.pos();  // Position des Touchpoints
            if (rect().contains(touchPos.toPoint())) {  // Überprüfen, ob der Touchpunkt im Widget-Bereich liegt
                int newValue = std::clamp(mapToSliderValue(touchPos.x()), 0, 100);  // Berechnet den neuen Wert basierend auf der X-Position
                setValue(newValue);
            }
        }
        return true;  // Touch-Ereignis wurde verarbeitet
    }
    return QWidget::event(event);  // Anderen Ereignissen Standardverhalten geben
}

void CustomTouchSliderHorizontal::mousePressEvent(QMouseEvent *event)
{
    if (rect().contains(event->pos())) {  // Überprüfen, ob der Mauszeiger im Widget-Bereich ist
        int newValue = mapToSliderValue(event->x());  // Berechnet den neuen Wert basierend auf der X-Position
        setValue(newValue);
    }
}

void CustomTouchSliderHorizontal::mouseMoveEvent(QMouseEvent *event)
{
    if (rect().contains(event->pos())) {  // Überprüfen, ob der Mauszeiger im Widget-Bereich ist
        int newValue = mapToSliderValue(event->x());  // Berechnet den neuen Wert basierend auf der X-Position
        setValue(newValue);
        qDebug() << "Value: " << newValue;
    }
}

void CustomTouchSliderHorizontal::mouseReleaseEvent(QMouseEvent *)
{
    // Optional: Hier kann man spezielle Logik nach dem Loslassen der Maus hinzufügen, falls gewünscht.
}

int CustomTouchSliderHorizontal::mapToSliderValue(int x)
{
    qreal dpi = this->devicePixelRatioF();

    const int margin = 10 * dpi;
    const int sliderWidth = 60 * dpi;
    int usableWidth = width() - 2 * margin - sliderWidth;

    // Korrigiere x: Mittelpunkt der Maus/Touchposition ist entscheidend
    x = std::clamp(x, margin + sliderWidth / 2, margin + usableWidth + sliderWidth / 2);
    int relativeX = x - margin - sliderWidth / 2;

    int value = (relativeX * 100) / usableWidth;
    return std::clamp(value, 0, 100);
}


void CustomTouchSliderHorizontal::setValue(int newValue)
{
    if (newValue != m_value) {
        m_value = newValue;

        // An Roboter publishen
        m_robot_node->publish_velocity(m_robot_node->getVelocity(), m_value);

        update();  // Widget neu zeichnen, um die Slider-Position zu aktualisieren
    }
}
