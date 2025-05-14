#include "../include/test_qt_pkg/joystick.h"
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
}

void JoystickWidget::resizeEvent(QResizeEvent *event)
{
    QSize newSize = event->size();
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
    emit positionChanged(normalizedPosition());
}

QPointF JoystickWidget::normalizedPosition() const
{
    QPointF delta = m_knobPos - m_center;
    QPointF normDelta = QPointF(delta.x() / m_maxRadius, delta.y() / m_maxRadius);
    qreal normDistance = std::hypot(normDelta.x(), normDelta.y());
    qreal angle = std::atan2(normDelta.y(), normDelta.x());
    qreal angleInDegrees = qRadiansToDegrees(angle);

    qDebug() << "Joystick Speed:" << normDistance << "Joystick Angle:" << angleInDegrees;
    return normDelta;
}
