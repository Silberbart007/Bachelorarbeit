#include "wheel.h"
#include <QPainter>
#include <QMouseEvent>
#include <QtMath>
#include <QDebug>
#include "rclcpp/rclcpp.hpp"

WheelWidget::WheelWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(150, 150);
    setAttribute(Qt::WA_AcceptTouchEvents, true);  // Touch-Ereignisse akzeptieren
}

void WheelWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Mittelpunkt
    QPointF center(width() / 2, height() / 2);

    // Außenradius: Pass auf, dass der Rand vom Stift berücksichtigt wird
    qreal outerRadius = qMin(width(), height()) / 2 - m_style.outerRingWidth;
    qreal innerRadius = m_style.centerCircleRadius;

    // Transformation auf Mittelpunkt
    painter.translate(center);
    painter.rotate(-m_currentAngle);

    // Außenring zeichnen
    painter.setPen(QPen(m_style.outerRingColor, m_style.outerRingWidth));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(QPointF(0, 0), outerRadius, outerRadius);

    // Speichen zeichnen
    painter.setPen(QPen(m_style.spokeColor, m_style.spokeWidth, Qt::SolidLine, Qt::RoundCap));
    for (int angle : m_style.spokeAnglesDegrees) {
        qreal angleRad = qDegreesToRadians(double(angle));
        QPointF end(
            outerRadius * std::cos(angleRad),
            outerRadius * std::sin(angleRad)
        );
        painter.drawLine(QPointF(0, 0), end);
    }

    // Mittelkreis zeichnen
    painter.setBrush(m_style.centerCircleColor);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(QPointF(0, 0), innerRadius, innerRadius);

    // Text im Mittelkreis
    if (!m_style.centerText.isEmpty()) {
        painter.setPen(Qt::black);  // Farbe kannst du ggf. auch im Style definieren
        painter.setFont(m_style.centerFont);
        QRectF textRect(-innerRadius, -innerRadius/2, innerRadius * 2, innerRadius);
        painter.drawText(textRect, Qt::AlignCenter, m_style.centerText);
    }
}




// Exakt wie bei Maus-Events, aber speziell für Touch
bool WheelWidget::event(QEvent *event)
{
    if (event->type() == QEvent::TouchBegin ||
        event->type() == QEvent::TouchUpdate ||
        event->type() == QEvent::TouchEnd)
    {
        QTouchEvent *touchEvent = static_cast<QTouchEvent *>(event);
        const QList<QTouchEvent::TouchPoint> touchPoints = touchEvent->touchPoints();

        if (!touchPoints.isEmpty()) {
            const QTouchEvent::TouchPoint &point = touchPoints.first();

            // Positionen bestimmen
            QPointF center(width() / 2, height() / 2);
            QPointF pos = point.pos();
            qreal dx = pos.x() - center.x();
            qreal dy = pos.y() - center.y();
            qreal angle = qAtan2(-dy, dx);  // Radiant

            // Lenkrad wird berührt
            if (event->type() == QEvent::TouchBegin) {
                m_startAngle = angle;
                m_lastAngle = m_currentAngle;
                m_isDragging = true;
            }
            // Lenkrad wird bewegt
            else if (event->type() == QEvent::TouchUpdate && m_isDragging) {
                qreal deltaDeg = (angle - m_startAngle) * 180.0 / M_PI;
                deltaDeg = qBound(-5.0, deltaDeg, 5.0); // Hier kann man 5.0 höher/runter machen, je nachdem wie schnell das Lenkrad drehen soll

                m_currentAngle = m_lastAngle + deltaDeg;
                m_currentAngle = qBound(-m_style.MaxAngle, m_currentAngle, m_style.MaxAngle); // Hier m_style.MaxAngle ändern, für kleineren/größeren Drehwinkel

                qDebug() << "Touch-Winkel: " << m_currentAngle;

                update();

                m_startAngle = angle;
                m_lastAngle = m_currentAngle;

                double normRot = -m_currentAngle / m_style.MaxAngle;
                m_robot_node->publish_velocity(m_robot_node->getSpeedNormalized(), normRot);
            }
            // Finger wird hochgehoben
            else if (event->type() == QEvent::TouchEnd) {
                m_isDragging = false;
            }
        }

        return true;  // Touch wurde behandelt
    }

    return QWidget::event(event);  // Weitergabe an Basis
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
        m_currentAngle = qBound(-m_style.MaxAngle, m_currentAngle, m_style.MaxAngle);

        qDebug() << "Lenkrad Winkel: " << m_currentAngle;

        update();

        m_startAngle = currentAngleRad;
        m_lastAngle = m_currentAngle;

        // Daten an Robot Node senden
        // Auf -1 bis 1 normieren
        double normRot = -m_currentAngle / m_style.MaxAngle;
        m_robot_node->publish_velocity(m_robot_node->getSpeedNormalized(), normRot);
    }
}


// Wenn Maus losgelassen wird
void WheelWidget::mouseReleaseEvent(QMouseEvent *)
{
    m_isDragging = false; // Ende des Ziehens
}

// Value (von außerhalb dieses Skripts) setzen
void WheelWidget::setValue(double newValue) {
    m_currentAngle = -newValue * m_style.MaxAngle;
    m_lastAngle = m_currentAngle;
    update();
}

// Wheelstyle setzen
void WheelWidget::setStyle(const WheelStyle &style) {
    m_style = style;
    update(); // neu zeichnen
}



