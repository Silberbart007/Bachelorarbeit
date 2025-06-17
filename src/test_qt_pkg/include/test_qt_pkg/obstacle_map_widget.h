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
    void setDrawPathMode(bool isEnabled) { m_drawPathMode = isEnabled; };
    void setBeamMode(bool isEnabled) { m_beamMode = isEnabled; };
    void setFollowMode(bool isEnabled) { m_followMode = isEnabled; };
    void setGhostMode(bool isEnabled) { m_ghostMode = isEnabled; };
    void setGhostDuration(double newDuration) { m_ghost_duration = newDuration; };
    void setGhostColor(QColor newColor) { m_ghost_color = newColor; };
    void setCurveGain(double newGain) { m_curve_gain = newGain; };
    void setLaserColor(QColor newColor) { m_beam_color = newColor; }; 
    void setLaserNumber(int newNumber) { m_laser_number = newNumber; };
    void setInertiaMode(bool isEnabled) { m_inertiaMode = isEnabled; };
    void setTrailMode(bool isEnabled) { m_trailMode = isEnabled; };
    void setTrailLifetime(double newLifetime) { m_trail_lifetime_ms = newLifetime*100; };
    void setTrailColor(QColor newColor) { m_trail_color = newColor; };

    // Alle Punkte und Paths von der Karte löschen
    void deleteAllDrawings();

    // Callback für Laser (wird in RobotNode aufgerufen)
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

protected:
    void resizeEvent(QResizeEvent *) override;
    //void paintEvent(QPaintEvent *) override; 

    // Zum Abfangen von Mausbewegungen über das viewport()
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots:
    void handleInertia();

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

    QGraphicsScene *m_scene;
    QGraphicsView *m_view;
    QGraphicsEllipseItem *m_robot_ellipse;
    QGraphicsLineItem *m_orientation_line;  // Linie für Richtung

    double m_robot_x_pixels;
    double m_robot_y_pixels;
    double m_robot_x_meters;
    double m_robot_y_meters;
    const double EMERGENCY_STOP_RADIUS = 3.0f;  // Radius in Pixeln
    double m_robot_theta_rad = 0.0; // Ausrichtung in Radiant
    double m_last_speed = 0.0;
    double m_last_steering = 0.0;
    const double m_max_steering = M_PI / 4;

    // Bool Variablen, ob Funktion gerade aktiv ist
    bool m_drawPathMode = false;
    bool m_beamMode = false;
    bool m_followMode = false;
    bool m_ghostMode = false;
    bool m_inertiaMode = false;
    bool m_trailMode = false;

    // Pfad zeichnen 
    bool m_drawing;
    QVector<QPointF> m_path_points;
    QGraphicsPathItem* m_temp_path_item;
    int m_current_target_index = 0;  // Index des aktuellen Zielpunkts
    QVector<QPointF> m_current_path; // Der Pfad, als Liste von Punkten
    QGraphicsEllipseItem* m_temp_point_item; // Einzelnen Punkt verfolgen

    // Follow Mode
    bool m_following;
    QPointF m_current_follow_point;

    // Ghost Mode
    std::vector<QGraphicsEllipseItem*> m_ghostItems;
    QTimer* m_ghost_timer = nullptr;
    int m_ghost_frame_index = 0;
    std::vector<Pose2D> m_ghost_trajectory;
    double m_wheel_base = 30.0;
    double m_ghost_duration = 2.0;
    double m_curve_gain = 1.25;
    QColor m_ghost_color;
    std::vector<Pose2D> computeGhostTrajectoryDiffDrive(
        double v,                       // Vorwärtsgeschwindigkeit in cm/s
        double omega,                   // Drehgeschwindigkeit in rad/s
        double duration_sec,            // Simulationsdauer
        int steps,                      // Simulationsschritte
        double theta_start_rad          // Anfangsorientierung
    );    
    void startGhostAnimation(double speed_cm_s, double steering_value);
    void updateGhostAnimation(double x_pos, double y_pos);
    void deleteGhosts();

    // Inertia Mode
    QPointF m_inertiaStart;
    QTime m_inertiaStartTime;
    QTimer m_inertiaTimer;
    QPointF m_inertiaVelocity;

    // Trail mode
    std::deque<QPair<QPointF, QTime>> m_trailHistory; // Position + Zeit
    QVector<QGraphicsLineItem*> m_trailLines; // Linien in der Szene
    const int TRAIL_MAX_POINTS = 30; // Maximal gespeicherte Punkte
    int m_trail_lifetime_ms = 2000; // Wie lange Punkte sichtbar bleiben
    QColor m_trail_color;
    void updateSpeedTrail(const QPointF& currentPosition);

    // Laserdaten
    QVector<QGraphicsLineItem*> m_beam_items;
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
