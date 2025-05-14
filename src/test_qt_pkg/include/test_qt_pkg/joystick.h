#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <QWidget>
#include <QPointF>

class JoystickWidget : public QWidget
{
    Q_OBJECT

public:
    explicit JoystickWidget(QWidget *parent = nullptr);

    QPointF normalizedPosition() const; // x und y von -1 bis +1

signals:
    void positionChanged(QPointF);

protected:
    void paintEvent(QPaintEvent *) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *) override;
    void resizeEvent(QResizeEvent *) override;

private:
    QPointF m_center;
    QPointF m_knobPos;
    float m_maxRadius; 
    bool m_dragging = false;

    void updateKnob(const QPointF &pos);
};

#endif // JOYSTICK_H
