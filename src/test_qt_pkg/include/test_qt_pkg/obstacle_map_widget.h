#ifndef OBSTACLEMAPWIDGET_H
#define OBSTACLEMAPWIDGET_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QTimer>
#include <QtMath>
#include <QDebug>
#include <algorithm>

namespace Ui {
class ObstacleMapWidget;
}

class ObstacleMapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ObstacleMapWidget(QWidget *parent = nullptr);
    ~ObstacleMapWidget();

    void addObstacle(int x, int y, int width, int height);
    void updateObstacles();
    void updateRobotPosition(float x, float y, float theta);
    void setupStaticObstacles();
    bool isNearObstacle(float x, float y);

protected:
    void resizeEvent(QResizeEvent *) override;

private:
    Ui::ObstacleMapWidget *ui;
    QGraphicsScene *scene_;
    QGraphicsView *view_;
    QGraphicsEllipseItem *robot_;
    QGraphicsLineItem *orientationLine_;  // Linie für Richtung

    float robot_x_ = 400;
    float robot_y_ = 300;
    const float EMERGENCY_STOP_RADIUS = 15.0f;  // Radius in Pixeln
    float robot_theta_ = 0.0; // Ausrichtung in Radiant
};

#endif // OBSTACLEMAPWIDGET_H
