#ifndef WHEEL_H
#define WHEEL_H

#include "robot_node.h"
#include "wheelstyle.h"
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPointF>
#include <QTouchEvent>
#include <QWidget>
#include <QTimer>

class WheelWidget : public QWidget {
    Q_OBJECT

  public:
    explicit WheelWidget(QWidget* parent = nullptr);
    ~WheelWidget() = default;

    // Robot Node setzen
    void setRobotNode(std::shared_ptr<RobotNode> robot_node) {
        m_robot_node = robot_node;
    };
    void setValue(double newValue);

    void setStyle(const WheelStyle& style);

  protected:
    // Paint-Event, um das Lenkrad zu zeichnen
    void paintEvent(QPaintEvent*) override;

    bool event(QEvent* event) override; // Zum Verarbeiten von Touch- und Maus-Ereignissen

    // Mausereignisse zum Steuern des Lenkrads
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent*) override;

  private:
    // Der aktuelle Winkel des Lenkrads in Grad
    qreal m_currentAngle = 0.0f;
    qreal m_startAngle = 0.0;
    qreal m_lastAngle = 0.0;

    // Zeiger aufs robot node
    std::shared_ptr<RobotNode> m_robot_node;

    // Flag, das angibt, ob die Maus gezogen wird
    bool m_isDragging = false;

    // Style des Lenkrads
    WheelStyle m_style;

    // For sending velocity while holding widget
    QTimer* m_velocityTimer;

    // For timer that sends current velocity while holding widget
    void sendCurrentVelocity();
};

#endif // WHEEL_H
