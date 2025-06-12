#ifndef TOUCH_SLIDER_VERTICAL_H
#define TOUCH_SLIDER_VERTICAL_H

#include <QWidget>
#include <QTouchEvent>
#include <QMouseEvent>
#include <QPainter>
#include "mainwindow.h"

class CustomTouchSliderVertical : public QWidget
{
    Q_OBJECT

public:
    explicit CustomTouchSliderVertical(QWidget *parent = nullptr);

    int getValue() const;  // Gibt den aktuellen Sliderwert zurück
    void setValue(double newValue);  // Setzt den Sliderwert und aktualisiert das Widget
    void setRobotNode(std::shared_ptr<RobotNode> robot_node) { m_robot_node = robot_node; };

protected:
    void paintEvent(QPaintEvent *) override;  // Zum Zeichnen des Sliders
    bool event(QEvent *event) override;  // Zum Verarbeiten von Touch- und Maus-Ereignissen
    void mousePressEvent(QMouseEvent *event) override;  // Verarbeitet Maus-Press-Ereignisse
    void mouseMoveEvent(QMouseEvent *event) override;  // Verarbeitet Maus-Bewegungs-Ereignisse
    void mouseReleaseEvent(QMouseEvent *) override;  // Optional für Release-Events

private:
    double m_value;  // Der aktuelle Wert des Sliders
    std::shared_ptr<RobotNode> m_robot_node;

    double mapToSliderValue(double y);  // Mapped die X-Position auf den Sliderwert
    double sliderValueToPixels(double value) const;
};

#endif // TOUCH_SLIDER_VERTICAL_H
