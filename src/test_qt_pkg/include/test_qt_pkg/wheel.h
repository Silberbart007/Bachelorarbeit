#ifndef WHEEL_H
#define WHEEL_H

#include <QWidget>
#include <QPointF>
#include <QPainter>

class WheelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit WheelWidget(QWidget *parent = nullptr);
    ~WheelWidget() = default;

signals:
    // Signal, das ausgelöst wird, wenn der Lenkradwinkel geändert wird
    void angleChanged(float angle);

protected:
    // Paint-Event, um das Lenkrad zu zeichnen
    void paintEvent(QPaintEvent *) override;

    // Mausereignisse zum Steuern des Lenkrads
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *) override;

private:
    // Der aktuelle Winkel des Lenkrads in Grad
    qreal m_currentAngle = 0.0f;
    qreal m_startAngle = 0.0;
    qreal m_lastAngle = 0.0;

    // Flag, das angibt, ob die Maus gezogen wird
    bool m_isDragging = false;
};

#endif // WHEEL_H
