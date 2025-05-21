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
    painter.setRenderHint(QPainter::TextAntialiasing);

    QPointF center(width() / 2, height() / 2);

    qreal outerRadius = qMin(width(), height()) / 2 - m_style.outerRingWidth;
    qreal innerRadius = m_style.centerCircleRadius;

    painter.translate(center);
    painter.rotate(-m_currentAngle);

    if (m_style.isRound) {
        // Klassisches rundes Lenkrad
        painter.setPen(QPen(m_style.outerRingColor, m_style.outerRingWidth));
        painter.setBrush(Qt::NoBrush);
        painter.drawEllipse(QPointF(0, 0), outerRadius, outerRadius);
    } else {
        // F1 Lenkrad zeichnen

        painter.setPen(QPen(m_style.outerRingColor, m_style.outerRingWidth));
        painter.setBrush(Qt::NoBrush);

        // Polygon mit abgeflachten Kanten statt Kreis
        QPolygonF wheelShape;

        // Definiere Punkte für abgeflachte Ober- und Unterseite

        // 8 Punkte, mit abgeflachter Ober- und Unterseite
        constexpr int n = 8;
        for (int i = 0; i < n; ++i) {
            // Winkel in Radiant (Start bei oben, im Uhrzeigersinn)
            qreal angle = (360.0 / n) * i;
            qreal angleRad = qDegreesToRadians(angle);

            // Radius je nach Position verändern, z.B. oben und unten flacher
            qreal radius = outerRadius;
            // Flach oben (bei 90°) und unten (bei 270°)
            if (angle >= 60 && angle <= 120) radius *= 0.7;   // oben abflachen
            if (angle >= 240 && angle <= 300) radius *= 0.7;  // unten abflachen

            QPointF pt(radius * std::cos(angleRad), radius * std::sin(angleRad));
            wheelShape << pt;
        }

        painter.drawPolygon(wheelShape);
    }

    // Speichen zeichnen
    painter.setPen(QPen(m_style.spokeColor, m_style.spokeWidth, Qt::SolidLine, Qt::RoundCap));
    for (int angle : m_style.spokeAnglesDegrees) {
        qreal angleRad = qDegreesToRadians(double(angle));
        
        // Radius für die Speiche berechnen, an abgeflachten Stellen kleiner setzen
        qreal spokeRadius = outerRadius;
        if (!m_style.isRound) {
            if (angle >= 60 && angle <= 120) spokeRadius *= 0.7;   // oben abgeflacht
            if (angle >= 240 && angle <= 300) spokeRadius *= 0.7;  // unten abgeflacht
        }

        QPointF end(
            spokeRadius * std::cos(angleRad),
            spokeRadius * std::sin(angleRad)
        );
        painter.drawLine(QPointF(0, 0), end);
    }

    // Mittelkreis
    painter.setBrush(m_style.centerCircleColor);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(QPointF(0, 0), innerRadius, innerRadius);

    // Text im Mittelkreis
    if (!m_style.centerText.isEmpty()) {
        painter.setPen(m_style.centerTextColor);
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
                m_currentAngle = qBound(-m_style.maxAngle, m_currentAngle, m_style.maxAngle); // Hier m_style.maxAngle ändern, für kleineren/größeren Drehwinkel

                qDebug() << "Touch-Winkel: " << m_currentAngle;

                update();

                m_startAngle = angle;
                m_lastAngle = m_currentAngle;

                double normRot = -m_currentAngle / m_style.maxAngle;
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
        m_currentAngle = qBound(-m_style.maxAngle, m_currentAngle, m_style.maxAngle);

        qDebug() << "Lenkrad Winkel: " << m_currentAngle;

        update();

        m_startAngle = currentAngleRad;
        m_lastAngle = m_currentAngle;

        // Daten an Robot Node senden
        // Auf -1 bis 1 normieren
        double normRot = -m_currentAngle / m_style.maxAngle;
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
    m_currentAngle = -newValue * m_style.maxAngle;
    m_lastAngle = m_currentAngle;
    update();
}

// Wheelstyle setzen
void WheelWidget::setStyle(const WheelStyle &style) {
    m_style = style;
    update(); // neu zeichnen
}



