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
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
    void setRobotNode(std::shared_ptr<RobotNode> robot_node);
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
    void initializeRobot();

    Ui::ObstacleMapWidget *ui;
    QGraphicsScene *scene_;
    QGraphicsView *view_;
    QGraphicsEllipseItem *robot_;
    QGraphicsLineItem *orientationLine_;  // Linie für Richtung

    double m_robot_x_pixels = 400;
    double m_robot_y_pixels = 300;
    double m_robot_x_meters = 400;
    double m_robot_y_meters = 300;
    const double EMERGENCY_STOP_RADIUS = 3.0f;  // Radius in Pixeln
    double robot_theta_ = 0.0; // Ausrichtung in Radiant

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
    std::vector<double> latestDistances_;
    QVector<QGraphicsLineItem*> beam_items_;

    // Roboter Node
    std::shared_ptr<RobotNode> m_robot_node;

    // Map 
    nav_msgs::msg::OccupancyGrid m_map;
    const double m_pixels_per_meter = 100.0;
    const double m_robot_size = 40.0;


};

#endif // OBSTACLEMAPWIDGET_H
