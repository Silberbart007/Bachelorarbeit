#include "joystick.h"
#include <QPainter>
#include <QMouseEvent>
#include <QtMath>
#include <QDebug>
#include "rclcpp/rclcpp.hpp"

JoystickWidget::JoystickWidget(QWidget *parent)
    : QWidget(parent)
{
    //setMinimumSize(150, 150);
    //setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_center = QPointF(width() / 2, height() / 2);
    m_knobPos = m_center;

    setAttribute(Qt::WA_AcceptTouchEvents, true);  // Touch-Ereignisse akzeptieren
}

void JoystickWidget::resizeEvent(QResizeEvent *event)
{
    //QSize newSize = event->size();
    //int side = qMin(newSize.width(), newSize.height());  // Wähle die kleinere Dimension
    //this->resize(side, side);  // Setze Breite und Höhe auf denselben Wert

    // Center neu berechnen
    m_center = QPointF(width() / 2, height() / 2);

    // Korrekte Knob-Position setzen
    if (!m_dragging)
        m_knobPos = m_center;
    QWidget::resizeEvent(event);  // Event weiterleiten
}

void JoystickWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Finde das größte Quadrat innerhalb des Widgets
    int side = qMin(width(), height());
    QPoint topLeft((width() - side) / 2, (height() - side) / 2);
    QRectF circleRect(topLeft, QSize(side, side));

    // Aktualisiere Zentrum und maximalen Radius
    m_center = QPointF(width() / 2, height() / 2);
    m_maxRadius = side / 2 - 20;  // 20 Pixel Puffer

    // Hintergrundkreis
    p.setBrush(QColor(200, 200, 200));
    p.drawEllipse(circleRect);

    // Knopf zeichnen
    p.setBrush(Qt::blue);
    p.drawEllipse(m_knobPos, 40, 40);
}

// Exakt wie Mausevents, aber hier für Touch
bool JoystickWidget::event(QEvent *event)
{
    if (event->type() == QEvent::TouchBegin || 
        event->type() == QEvent::TouchUpdate || 
        event->type() == QEvent::TouchEnd) {

        QTouchEvent *touchEvent = static_cast<QTouchEvent *>(event);
        const QList<QTouchEvent::TouchPoint> &touchPoints = touchEvent->touchPoints();

        if (!touchPoints.isEmpty()) {
            const QTouchEvent::TouchPoint &point = touchPoints.first();
            QPointF touchPos = point.pos();

            if (event->type() == QEvent::TouchBegin) {
                if ((touchPos - m_knobPos).manhattanLength() < 40) {
                    m_dragging = true;
                }
            }

            if (event->type() == QEvent::TouchUpdate) {
                if (m_dragging) {
                    updateKnob(touchPos);
                }
            }

            if (event->type() == QEvent::TouchEnd) {
                m_dragging = false;
                updateKnob(m_center);  // zurück zur Mitte
            }
        }

        return true;  // Touch-Ereignis verarbeitet
    }

    return QWidget::event(event);  // Standardverhalten für andere Events
}


void JoystickWidget::mousePressEvent(QMouseEvent *event)
{
    if ((event->pos() - m_knobPos).manhattanLength() < 40) {
        m_dragging = true;
    }
}

void JoystickWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_dragging) {
        updateKnob(event->pos());
    }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent *)
{
    m_dragging = false;
    updateKnob(m_center);  // zurück zur Mitte
}

void JoystickWidget::updateKnob(const QPointF &pos)
{
    QPointF delta = pos - m_center;
    qreal distance = std::hypot(delta.x(), delta.y());

    if (distance > m_maxRadius) {
        delta *= (m_maxRadius / distance);
    }

    m_knobPos = m_center + delta;
    update();

    QPointF norm = normalizedPosition();

    double x_speed = -norm.y();  // vertikal invertiert → vorwärts/rückwärts
    double y_speed = -norm.x();  // horizontal invertiert

    RobotNode::RobotSpeed speed = {x_speed, y_speed};
    m_robot_node->publish_velocity(speed, m_robot_node->getRotationNormalized());
}

QPointF JoystickWidget::normalizedPosition() const
{
    QPointF delta = m_knobPos - m_center;
    QPointF normDelta = QPointF(delta.x() / m_maxRadius, delta.y() / m_maxRadius);
    return normDelta;
}

void JoystickWidget::setValue(const RobotNode::RobotSpeed &speed)
{
    QPointF pos;
    pos.setX(m_center.x() - speed.y * m_maxRadius);  // horizontal invertiert
    pos.setY(m_center.y() - speed.x * m_maxRadius);  // vertikal invertiert

    m_knobPos = pos;
    update();
}
