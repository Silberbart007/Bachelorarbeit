#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "robot_node.h"
#include <QPointF>
#include <QWidget>
#include <QTimer>

class JoystickWidget : public QWidget {
    Q_OBJECT

  public:
    explicit JoystickWidget(QWidget* parent = nullptr);

    QPointF normalizedPosition() const; // x und y von -1 bis +1
    void setRobotNode(std::shared_ptr<RobotNode> robot_node) {
        m_robot_node = robot_node;
    };
    void setValue(const RobotNode::RobotSpeed& speed);
    void setOmni(bool isEnabled) {
        m_omni = isEnabled;
    }

  protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void resizeEvent(QResizeEvent*) override;

    bool event(QEvent* event) override; // Zum Verarbeiten von Touch- und Maus-Ereignissen

  private:
    QPointF m_center;
    QPointF m_knobPos;
    float m_maxRadius;
    bool m_dragging = false;
    bool m_omni = true;

    std::shared_ptr<RobotNode> m_robot_node;

    // For sending velocity while holding widget
    QTimer* m_velocityTimer;

    // For timer that sends current velocity while holding widget
    void sendCurrentVelocity();

    void updateKnob(const QPointF& pos);
};

#endif // JOYSTICK_H
