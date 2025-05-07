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
    QVector<qreal> spokeAngles = { 0, 120, 240 };

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

        // Winkel in Bereich [-180°, +180°] normalisieren
        while (deltaDeg > 180) deltaDeg -= 360;
        while (deltaDeg < -180) deltaDeg += 360;

        qreal newAngle = m_lastAngle + deltaDeg;

        // Begrenzung des Winkels auf den gewünschten Bereich, falls gewünscht
        newAngle = qBound(-180.0, newAngle, 180.0);

        // Optional: Begrenzen auf sinnvolle Werte, z. B. [-180°, +180°]
        if (newAngle > 180) newAngle = 180;
        if (newAngle < -180) newAngle = -180;

        qDebug() << "newAngle: " << newAngle;

        // Nur aktualisieren, wenn sich der Winkel verändert
        if (!qFuzzyCompare(newAngle, m_currentAngle)) {
            m_currentAngle = newAngle;
            update();
            emit angleChanged(m_currentAngle);
        }

    }
}



void WheelWidget::mouseReleaseEvent(QMouseEvent *)
{
    m_isDragging = false; // Ende des Ziehens
}



