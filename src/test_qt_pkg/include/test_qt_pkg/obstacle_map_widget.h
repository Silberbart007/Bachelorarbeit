#ifndef OBSTACLEMAPWIDGET_H
#define OBSTACLEMAPWIDGET_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QTimer>
#include <QtMath>
#include <QDebug>
#include <QMouseEvent>
#include <algorithm>
#include "robot_node.h"

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
    void updateRobotPosition(double x, double y, double theta);
    void setupStaticObstacles();
    bool isNearObstacle(float x, float y);

    void setRobotNode(std::shared_ptr<RobotNode> robot_node) { m_robot_node = robot_node; };

protected:
    void resizeEvent(QResizeEvent *) override;

    // Zum Abfangen von Mausbewegungen über das viewport()
    bool eventFilter(QObject *obj, QEvent *event) override;

private:
    void goToNextPoint();
    void pathDrawn(const QVector<QPointF>& points);
    QVector<QPointF> resamplePath(const QVector<QPointF>& originalPoints, double spacing);

    Ui::ObstacleMapWidget *ui;
    QGraphicsScene *scene_;
    QGraphicsView *view_;
    QGraphicsEllipseItem *robot_;
    QGraphicsLineItem *orientationLine_;  // Linie für Richtung

    float robot_x_ = 400;
    float robot_y_ = 300;
    const float EMERGENCY_STOP_RADIUS = 2.0f;  // Radius in Pixeln
    float robot_theta_ = 0.0; // Ausrichtung in Radiant

    // Pfad zeichnen 
    bool drawing_;
    QVector<QPointF> path_points_;
    QGraphicsPathItem* temp_path_item_;
    int current_target_index_ = 0;  // Index des aktuellen Zielpunkts
    QVector<QPointF> current_path_; // Der Pfad, als Liste von Punkten

    std::shared_ptr<RobotNode> m_robot_node;

};

#endif // OBSTACLEMAPWIDGET_H
