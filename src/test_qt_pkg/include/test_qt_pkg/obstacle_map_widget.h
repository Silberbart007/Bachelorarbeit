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
#include "nav2client.h"
#include "mainwindow.h"

class Nav2Client;

namespace Ui {
class ObstacleMapWidget;
}

class ObstacleMapWidget : public QWidget
{
    Q_OBJECT

public:
    // Hilfskonstruktion
    struct Pose2D {
        double x;
        double y;
        double theta;
    };

    explicit ObstacleMapWidget(QWidget *parent = nullptr);
    ~ObstacleMapWidget();

    // Setter
    void setRobotNode(std::shared_ptr<RobotNode> robot_node);
    void setNav2Node(std::shared_ptr<Nav2Client> nav2_node);
    void setDrawPathMode(bool isEnabled) { drawPathMode_ = isEnabled; };
    void setBeamMode(bool isEnabled) { beamMode_ = isEnabled; };
    void setFollowMode(bool isEnabled) { followMode_ = isEnabled; };
    void setGhostMode(bool isEnabled) { ghostMode_ = isEnabled; };
    void setGhostDuration(double newDuration) { m_ghost_duration = newDuration; };
    void setCurveGain(double newGain) { m_curve_gain = newGain; };
    void setLaserColor(QColor newColor) { m_beam_color = newColor; }; 
    void setLaserNumber(int newNumber) { m_laser_number = newNumber; };
    void setInertiaMode(bool isEnabled) { inertiaMode_ = isEnabled; };

    // Alle Punkte und Paths von der Karte löschen
    void deleteAllDrawings();

    // Callback für Laser (wird in RobotNode aufgerufen)
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

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
    void addObstacle(double x, double y, double width, double height);
    void updateObstacles();
    void updateRobotPosition(double x, double y, double theta);
    void setupStaticObstacles();
    bool isNearObstacle(float x, float y);
    void followCurrentPoint();
    void initializeRobot();

    // Hilfsfunktionen zum umrechnen
    QPointF worldToScene(double x_m, double y_m);
    QPointF sceneToMapCoordinates(const QPointF& scene_pos);

    // Beams aus Laserdaten generieren
    void generateLaserBeams();

    Ui::ObstacleMapWidget *ui;
    QGraphicsScene *scene_;
    QGraphicsView *view_;
    QGraphicsEllipseItem *robot_;
    QGraphicsLineItem *orientationLine_;  // Linie für Richtung

    double m_robot_x_pixels;
    double m_robot_y_pixels;
    double m_robot_x_meters;
    double m_robot_y_meters;
    const double EMERGENCY_STOP_RADIUS = 3.0f;  // Radius in Pixeln
    double robot_theta_ = 0.0; // Ausrichtung in Radiant
    double m_last_speed = 0.0;
    double m_last_steering = 0.0;
    const double m_max_steering = M_PI / 4;

    // Bool Variablen, ob Funktion gerade aktiv ist
    bool drawPathMode_ = false;
    bool beamMode_ = false;
    bool followMode_ = false;
    bool ghostMode_ = false;
    bool inertiaMode_ = false;

    // Pfad zeichnen 
    bool drawing_;
    QVector<QPointF> path_points_;
    QGraphicsPathItem* temp_path_item_;
    int current_target_index_ = 0;  // Index des aktuellen Zielpunkts
    QVector<QPointF> current_path_; // Der Pfad, als Liste von Punkten
    QGraphicsEllipseItem* temp_point_item_; // Einzelnen Punkt verfolgen

    // Follow Mode
    bool following_;
    QPointF current_follow_point_;

    // Ghost Mode
    std::vector<QGraphicsEllipseItem*> ghostItems_;
    QTimer* ghost_timer_ = nullptr;
    int ghost_frame_index_ = 0;
    std::vector<Pose2D> ghost_trajectory_;
    double m_wheel_base = 30.0;
    double m_ghost_duration = 2.0;
    double m_curve_gain = 1.25;
    std::vector<Pose2D> computeGhostTrajectoryDiffDrive(
        double v,                       // Vorwärtsgeschwindigkeit in cm/s
        double omega,                   // Drehgeschwindigkeit in rad/s
        double duration_sec,            // Simulationsdauer
        int steps,                      // Simulationsschritte
        double theta_start_rad          // Anfangsorientierung
    );    
    void startGhostAnimation(double speed_cm_s, double steering_value, double max_angle_rad, double wheel_base_cm);
    void updateGhostAnimation(double x_pos, double y_pos);
    void deleteGhosts();

    // Inertia Mode
    QPointF inertiaStart_;
    QTime inertiaStartTime_;
    QTimer* inertiaTimer_;
    QPointF currentVelocity_;

    // Skalierung mit Fingern
    QPointF lastPinchCenterInView_;
    QPointF lastPinchCenterInScene_;
    qreal lastPinchScaleFactor_;

    // Laserdaten
    // Dummy
    std::vector<double> latestDistances_;
    QVector<QGraphicsLineItem*> beam_items_;
    // Echter Laser
    sensor_msgs::msg::LaserScan m_current_scan;
    bool m_scan_available = false;
    QColor m_beam_color;
    int m_laser_number;

    // Roboter Node
    std::shared_ptr<RobotNode> m_robot_node;

    // Nav2 Node
    std::shared_ptr<Nav2Client> m_nav2_node;

    // Map 
    nav_msgs::msg::OccupancyGrid m_map;
    const double m_pixels_per_meter = 100.0;
    const double m_robot_size = 40.0;


};

#endif // OBSTACLEMAPWIDGET_H
