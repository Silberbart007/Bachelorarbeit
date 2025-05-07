#include "../include/test_qt_pkg/joystick.h"
#include <QPainter>
#include <QMouseEvent>
#include <QtMath>
#include <QDebug>
#include "rclcpp/rclcpp.hpp"

JoystickWidget::JoystickWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(150, 150);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_center = QPointF(width() / 2, height() / 2);
    m_knobPos = m_center;
}

void JoystickWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Hintergrundkreis
    p.setBrush(QColor(200, 200, 200));
    p.drawEllipse(rect());

    // Knopf (Joystick-Position)
    p.setBrush(Qt::blue);
    p.drawEllipse(m_knobPos, 20, 20);
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
    // Begrenzung auf Kreisradius
    qreal radius = width() / 2 - 20;
    QPointF delta = pos - m_center;
    qreal distance = std::hypot(delta.x(), delta.y());
    if (distance > radius) {
        delta *= (radius / distance);
    }

    m_knobPos = m_center + delta;
    update();

    emit positionChanged(normalizedPosition());
}

QPointF JoystickWidget::normalizedPosition() const
{
    QPointF delta = m_knobPos - m_center;
    qreal maxRadius = width() / 2 - 20;

    // Normalisiertes Delta
    QPointF normDelta = QPointF(delta.x() / maxRadius, delta.y() / maxRadius);

    // Normalisierte Distanz
    qreal normDistance = std::hypot(normDelta.x(), normDelta.y());

    // Berechnung der Bewegungsrichtung (Winkel)
    qreal angle = std::atan2(normDelta.y(), normDelta.x());
    qreal angleInDegrees = qRadiansToDegrees(angle);  // Umrechnung in Grad

    // Ausgabe von Geschwindigkeit und Winkel
    qDebug() << "Joystick Speed:" << normDistance << "Joystick Angle:" << angleInDegrees;

    return QPointF(delta.x() / maxRadius, delta.y() / maxRadius);
}
