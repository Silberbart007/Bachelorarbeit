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
#include <QRandomGenerator>
#include <QVBoxLayout>
#include <QGesture>
#include <QScrollBar>
#include <QImage>
#include "robot_node.h"
#include "mainwindow.h"

namespace Ui {
class ObstacleMapWidget;
}

class ObstacleMapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ObstacleMapWidget(QWidget *parent = nullptr);
    ~ObstacleMapWidget();

    // Setter
    void setRobotNode(std::shared_ptr<RobotNode> robot_node) { m_robot_node = robot_node; };
    void setDrawPathMode(bool isEnabled) { drawPathMode_ = isEnabled; };
    void setBeamMode(bool isEnabled) { beamMode_ = isEnabled; };

protected:
    void resizeEvent(QResizeEvent *) override;
    //void paintEvent(QPaintEvent *) override; 

    // Zum Abfangen von Mausbewegungen über das viewport()
    bool eventFilter(QObject *obj, QEvent *event) override;

private:
    void goToNextPoint();
    void pathDrawn(const QVector<QPointF>& points);
    void generateDummyData();
    void updateObstaclesFromMap();
    QVector<QPointF> resamplePath(const QVector<QPointF>& originalPoints, double spacing);
    void addObstacle(int x, int y, int width, int height);
    void updateObstacles();
    void updateRobotPosition(double x, double y, double theta);
    void setupStaticObstacles();
    bool isNearObstacle(float x, float y);
    QPointF worldToScene(double x_m, double y_m);

    Ui::ObstacleMapWidget *ui;
    QGraphicsScene *scene_;
    QGraphicsView *view_;
    QGraphicsEllipseItem *robot_;
    QGraphicsLineItem *orientationLine_;  // Linie für Richtung

    float robot_x_ = 400;
    float robot_y_ = 300;
    const float EMERGENCY_STOP_RADIUS = 2.0f;  // Radius in Pixeln
    float robot_theta_ = 0.0; // Ausrichtung in Radiant

    // Bool Variablen, ob Funktion gerade aktiv ist
    bool drawPathMode_ = false;
    bool beamMode_ = false;

    // Pfad zeichnen 
    bool drawing_;
    QVector<QPointF> path_points_;
    QGraphicsPathItem* temp_path_item_;
    int current_target_index_ = 0;  // Index des aktuellen Zielpunkts
    QVector<QPointF> current_path_; // Der Pfad, als Liste von Punkten
    QGraphicsEllipseItem* temp_point_item_; // Einzelnen Punkt verfolgen

    // Skalierung mit Fingern
    QPointF lastPinchCenterInView_;
    QPointF lastPinchCenterInScene_;
    qreal lastPinchScaleFactor_;

    // Laserdaten
    std::vector<float> latestDistances_;
    QVector<QGraphicsLineItem*> beam_items_;

    // Roboter Node
    std::shared_ptr<RobotNode> m_robot_node;

    // Map 
    nav_msgs::msg::OccupancyGrid m_map;

};

#endif // OBSTACLEMAPWIDGET_H
