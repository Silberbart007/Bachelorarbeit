#include "../include/test_qt_pkg/wheel.h"
#include <QPainter>
#include <QMouseEvent>
#include <QtMath>
#include <QDebug>
#include "rclcpp/rclcpp.hpp"

WheelWidget::WheelWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(150, 150);
}

void WheelWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Mittelpunkt und Radius
    QPointF center(width() / 2, height() / 2);
    qreal radius = qMin(width(), height()) / 2 - 10;

    // Hintergrund-Kreis
    painter.setBrush(QBrush(Qt::lightGray));
    painter.drawEllipse(center, radius, radius);

    // Winkel in Radiant
    qreal angleRad = qDegreesToRadians(m_currentAngle);

    // Liste der Speichenwinkel relativ zum aktuellen Winkel (z.B. 0°, 120°, 240°)
    QVector<qreal> spokeAngles = { 0+28, 120+28, 240+28 };

    painter.setPen(QPen(Qt::black, 3));

    for (qreal spoke : spokeAngles) {
        qreal totalAngle = angleRad + qDegreesToRadians(spoke);
        QPointF end(
            center.x() + radius * std::cos(totalAngle),
            center.y() - radius * std::sin(totalAngle)
        );
        painter.drawLine(center, end);
    }
}


// Wenn Maus gedrückt wird
void WheelWidget::mousePressEvent(QMouseEvent *event)
{
    QPointF center(width() / 2, height() / 2);
    QPointF pos = event->pos();
    qreal dx = pos.x() - center.x();
    qreal dy = pos.y() - center.y();

    m_startAngle = qAtan2(-dy, dx); // in Radiant
    m_lastAngle = m_currentAngle;  // in Grad
    m_isDragging = true;
}


// Wenn Maus bewegt wird
void WheelWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_isDragging) {
        QPointF center(width() / 2, height() / 2);
        QPointF pos = event->pos();
        qreal dx = pos.x() - center.x();
        qreal dy = pos.y() - center.y();
        qreal currentAngleRad = qAtan2(-dy, dx);

        // Berechne Delta in Grad
        qreal deltaDeg = (currentAngleRad - m_startAngle) * 180.0 / M_PI;

        // Änderung eingrenzen
        deltaDeg = qBound(-5.0, deltaDeg, 5.0);

        m_currentAngle = m_lastAngle + deltaDeg;

        // Maximaler Lenkrad-Winkel begrenzen
        m_currentAngle = qBound(-450.0, m_currentAngle, 450.0);

        qDebug() << "Lenkrad Winkel: " << m_currentAngle;

        update();
        emit angleChanged(m_currentAngle);

        m_startAngle = currentAngleRad;
        m_lastAngle = m_currentAngle;
    }
}


// Wenn Maus losgelassen wird
void WheelWidget::mouseReleaseEvent(QMouseEvent *)
{
    m_isDragging = false; // Ende des Ziehens
}



