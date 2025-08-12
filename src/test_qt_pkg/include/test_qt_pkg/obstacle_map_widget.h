/**
 * @file obstacle_map_widget.h
 * @brief Widget managing the obstacle map and multiple control modes.
 *
 * This widget inherits from QWidget and contains all logic and rendering related to
 * robot movement, visualization of sensor data, and user interaction with the map,
 * including support for modes like path drawing, inertia simulation, ghost prediction, etc.
 *
 * @author Max Vtulkin
 * @date 2025
 */

#ifndef OBSTACLEMAPWIDGET_H
#define OBSTACLEMAPWIDGET_H

// Qt-includes
#include <QDebug>
#include <QGesture>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QImage>
#include <QMouseEvent>
#include <QRandomGenerator>
#include <QScrollBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QtMath>

// ROS2-includes
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++-includes
#include <algorithm>

// Other own files
#include "mainwindow.h"
#include "nav2client.h"
#include "robot_node.h"
#include "scan_zone.h"

// Forward declaration of Nav2Client, to get the program to run properly
class Nav2Client;
class RobotNode;

// Needed for obstacle_map_widget_ui.h file
namespace Ui {
class ObstacleMapWidget;
}

/**
 * @class ObstacleMapWidget
 * @brief Widget that manages the obstacle map, robot movement, and multiple control modes.
 *
 * This widget inherits from QWidget and handles rendering and interaction for
 * robot navigation and environment visualization. It supports various control modes
 * including path drawing, inertia simulation, ghost prediction, and sensor data display.
 *
 * The widget processes user input (mouse, touch, gestures) and integrates with ROS2
 * to communicate with robot nodes. It also manages timers and graphical elements
 * such as trails, obstacles, and robot position indicators.
 */
class ObstacleMapWidget : public QWidget {
    Q_OBJECT

  public:
    /**
     * @brief Simple structure representing a 2D pose.
     *
     * Contains x and y coordinates (in meters) and orientation theta (in radians).
     */
    struct Pose2D {
        double x;
        double y;
        double theta;
    };

    struct ViewData {
        QPointF pos;
        qreal rot;
        qreal zoom;
    };

    // ====== Constructor / Destructor ======
    explicit ObstacleMapWidget(QWidget* parent = nullptr);
    ~ObstacleMapWidget();

    // ====== Node Setter ======
    /**
     * @brief Set the RobotNode instance used for robot communication.
     * @param robot_node Shared pointer to the RobotNode.
     */
    void setRobotNode(std::shared_ptr<RobotNode> robot_node);

    /**
     * @brief Set the Nav2Client instance used for navigation commands.
     * @param nav2_node Shared pointer to the Nav2Client.
     */
    void setNav2Node(std::shared_ptr<Nav2Client> nav2_node);

    // ====== Mode Setters ======
    /**
     * @brief Enable or disable various control modes.
     * The modes control different behaviors like drawing paths, laser beam visualization,
     * following paths, ghost trajectory prediction, inertia simulation, and trail visualization.
     */
    void setDrawPathMode(bool isEnabled) {
        m_drawPathMode = isEnabled;
    }
    void setBeamMode(bool isEnabled) {
        m_beamMode = isEnabled;
    }
    void setFollowMode(bool isEnabled) {
        m_followMode = isEnabled;
    }
    void setGhostMode(bool isEnabled) {
        m_ghostMode = isEnabled;
    }
    void setInertiaMode(bool isEnabled) {
        m_inertiaMode = isEnabled;
    }
    void setTrailMode(bool isEnabled) {
        m_trailMode = isEnabled;
    }
    void setCollisionBorderMode(bool isEnabled) {
        m_collisionBorderMode = isEnabled;
    }
    void setZoneMode(bool isEnabled) {
        m_zoneMode = isEnabled;
    }
    void setMapFollow(bool isEnabled) {
        m_mapFollow = isEnabled;
    }

    // ====== Parameter Setters ======
    /**
     * @brief Set parameters for certain modes such as trail lifetime, colors, durations, gains, and
     * laser settings.
     */
    void setTrailLifetime(double newLifetime) {
        m_trail_lifetime_ms = newLifetime * 1000;
    }
    void setTrailColor(QColor newColor) {
        m_trail_color = newColor;
    }
    void setGhostDuration(double newDuration) {
        m_ghost_duration = newDuration;
    }
    void setGhostColor(QColor newColor) {
        m_ghost_color = newColor;
    }
    void setCurveGain(double newGain) {
        m_curve_gain = newGain;
    }
    void setLaserColor(QColor newColor) {
        m_beam_color = newColor;
    }
    void setLaserNumber(int newNumber) {
        m_laser_number = newNumber;
    }

    // ====== Getter methods ======
    float getMinLaserDistance() {
        return m_min_laser_distance;
    }

    Pose2D getRobotPositionMeters() {
        return {m_robot_x_meters, m_robot_y_meters, m_robot_theta_rad};
    }

    QGraphicsView* view() const {
        return m_view;
    }

    ViewData getViewData() const;

    // ====== Other public methods ======

    void stopInertia();

    /**
     * @brief Callback for handling new laser scan data (called in RobotNode).
     * @param msg Shared pointer to the received LaserScan message.
     */
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // ===== Public slot methods =====
  public slots:
    /**
     * @brief Removes all user-drawn content (from drawPathMode) from the map.
     */
    void deleteAllDrawings();

  protected:
    /**
     * @brief Handles resizing of the widget.
     *
     * Called automatically when the size of the MainWindow or the obstacle map widget changes.
     * Should update internal layouts or scaling as needed.
     * @param event Pointer to the resize event containing new size info.
     */
    void resizeEvent(QResizeEvent* event) override;

    /**
     * @brief Event filter to intercept mouse and touch events.
     *
     * Filters events from the viewport to handle user interactions like clicks, gestures, or
     * touches.
     * @param obj The object that sent the event.
     * @param event The event to be processed.
     * @return true if the event is handled and should not propagate further; false otherwise.
     */
    bool eventFilter(QObject* obj, QEvent* event) override;

  private slots:
    /**
     * @brief Slot triggered periodically to handle inertia simulation.
     *
     * Connected to a QTimer, this function updates the speed and motion of the robot or
     * visualization, for example to simulate slowing down due to inertia.
     */
    void handleInertia();

  private:
    // ===== Scene and View Rendering =====

    /// Scene that holds all graphical elements
    QGraphicsScene* m_scene;

    /// View that displays the scene
    QGraphicsView* m_view;

    /// Robot visual representation (position)
    QGraphicsEllipseItem* m_robot_ellipse;

    /// Line indicating robot orientation
    QGraphicsLineItem* m_orientation_line;

    /// Last rotation angle for view rotation
    qreal m_lastRotationAngle_view = 0.0;

    /// Current scale of view
    qreal m_currentScale_view = 1.0;

    /// Current rotation of view
    qreal m_currentRotation_view = 0.0;

    /// Initial Scale of view
    qreal m_initialScale_view = 1.0;

    /// Initial Rotation of view
    qreal m_initialRotation_view = 0.0;

    /// Remember start rotation of pinch gesture
    qreal m_startPinchRotation = 0.0;

    /// Remember last touch position
    QPointF m_lastTouchPos;

    /// Track panning
    bool m_touchPanningActive = false;

    // ===== Robot State and Parameters =====

    /// Robot position in pixels
    double m_robot_x_pixels = 0.0;
    double m_robot_y_pixels = 0.0; // Down = positive, up = negative

    /// Robot position in meters
    double m_robot_x_meters = 0.0;
    double m_robot_y_meters = 0.0;

    /// Robot orientation in radians
    double m_robot_theta_rad = 0.0;

    /// Last movement commands
    double m_last_speed = 0.0;
    double m_last_steering = 0.0;

    /// Maximum steering angle in radians
    const double m_max_steering = M_PI / 4;

    /// Emergency stop radius (in pixels)
    const double EMERGENCY_STOP_RADIUS = 3.0f;

    // ===== Active Modes =====

    bool m_drawPathMode = false;
    bool m_beamMode = false;
    bool m_followMode = false;
    bool m_ghostMode = false;
    bool m_inertiaMode = false;
    bool m_trailMode = false;
    bool m_collisionBorderMode = false;
    bool m_zoneMode = false;
    bool m_mapFollow = false;

    // ===== Path Drawing Mode =====

    /// True if currently drawing path
    bool m_drawing = false;

    /// Collected path points during drawing
    QVector<QPointF> m_path_points;

    /// Full path currently in use
    QVector<QPointF> m_current_path;

    /// Current index in the active path
    int m_current_target_index = 0;

    /// Temporary path rendering item
    QGraphicsPathItem* m_temp_path_item = nullptr;

    /// Ellipse marking the current target point
    QGraphicsEllipseItem* m_temp_point_item = nullptr;

    // ===== Follow Mode =====

    /// True if robot is actively following a path
    bool m_following = false;

    /// Current target follow point
    QPointF m_current_follow_point;

    // ===== Ghost Mode (Trajectory Prediction) =====

    /// Ellipses showing ghost path
    std::vector<QGraphicsEllipseItem*> m_ghostItems;

    /// Timer controlling ghost animation
    QTimer* m_ghost_timer = nullptr;

    /// Current frame index in ghost animation
    int m_ghost_frame_index = 0;

    /// Predicted trajectory for ghost
    std::vector<Pose2D> m_ghost_trajectory;

    /// Duration of ghost prediction (seconds)
    double m_ghost_duration;

    /// Gain factor for curved motion
    double m_curve_gain;

    /// Color of ghost path
    QColor m_ghost_color;

    // ===== Inertia Mode =====

    /// Starting point of inertia simulation
    QPointF m_inertiaStart;

    /// Timestamp when inertia started
    QTime m_inertiaStartTime;

    /// Timer for updating inertia
    QTimer m_inertiaTimer;

    /// Velocity vector for inertia simulation
    QPointF m_inertiaVelocity;

    // ===== Trail Mode (Speed Trace) =====

    /// Position history with timestamps
    std::deque<QPair<QPointF, QTime>> m_trailHistory;

    /// Lines representing the trail in the scene
    QVector<QGraphicsLineItem*> m_trailLines;

    /// The main trail item
    QGraphicsPathItem* m_trailItem = nullptr;

    /// Max number of points in trail history
    const int TRAIL_MAX_POINTS = 30;

    /// Lifetime of trail points (ms)
    int m_trail_lifetime_ms = 2000;

    /// Color of trail lines
    QColor m_trail_color;

    // ===== Laser Beam Visualization =====

    /// Items representing individual laser beams
    QVector<QGraphicsLineItem*> m_beam_items;

    /// Latest scan data
    sensor_msgs::msg::LaserScan m_current_scan;

    /// True if a scan is available for rendering
    bool m_scan_available = false;

    /// Color used for laser beams
    QColor m_beam_color;

    /// Number of laser beams to display
    int m_laser_number;

    /// Smallest laser distance
    float m_min_laser_distance = 0.0f;

    // ===== Zone Mode =====

    /// Indicates whether a zone drawing gesture is currently active.
    bool m_zoneDrawingInProgress = false;

    /// The currently active scan zone being drawn.
    ScanZone m_activeZone;

    /// The number of fingers detected at the beginning of the gesture.
    int m_activeFingerCount = 0;

    /// The starting point of the touch gesture (center of the zone).
    QPointF m_touchStartCenter;

    /// The current radius of the scan zone.
    qreal m_touchRadius = 0.0;

    /// Initial Rotation of zone
    qreal m_initialRotationAngle_zone = 0;

    /// Current rotation of zone
    qreal m_currentRotationAngle_zone = 0;

    /// A list of all completed scan zones.
    QList<ScanZone> m_savedZones;

    // ===== ROS and Navigation Nodes =====

    /// Custom ROS node for robot communication
    std::shared_ptr<RobotNode> m_robot_node;

    /// Client for Nav2 navigation commands
    std::shared_ptr<Nav2Client> m_nav2_node;

    // ===== Map and Environment =====

    /// Occupancy grid map
    nav_msgs::msg::OccupancyGrid m_map;

    /// Scale factor: pixels per meter
    const double m_pixels_per_meter = 50.0;

    /// Size of robot in pixels
    const double m_robot_size = 20.0;

    /// Follow mode of obstacle map | true = center on robot
    bool m_follow_robot_on_map = true;

    /// Check if amcl data is received
    bool has_amcl = false;

    // ===== Support Functions =====

    /// \name Support Functions
    /// @{

    /**
     * @brief Initializes robot pose and view.
     */
    void initializeRobot();

    /**
     * @brief Moves to the next point in the current path.
     */
    void goToNextPoint();

    /**
     * @brief Called when a new path is drawn by the user.
     * @param points The points defining the new path.
     */
    void pathDrawn(const QVector<QPointF>& points);

    /**
     * @brief Extracts obstacles from the current occupancy map.
     */
    void updateObstaclesFromMap();

    /**
     * @brief Resamples a path to ensure evenly spaced points.
     * @param originalPoints Original path points.
     * @param spacing Desired spacing between points.
     * @return Resampled path points.
     */
    QVector<QPointF> resamplePath(const QVector<QPointF>& originalPoints, double spacing);

    /**
     * @brief Adds a static obstacle to the map.
     * @param x X position of the obstacle.
     * @param y Y position of the obstacle.
     * @param width Width of the obstacle.
     * @param height Height of the obstacle.
     */
    void addObstacle(double x, double y, double width, double height);

    /**
     * @brief Updates obstacle representation in the scene.
     */
    void updateObstacles();

    /**
     * @brief Updates robot pose in pixels and meters.
     * @param x X position in meters.
     * @param y Y position in meters.
     * @param theta Orientation angle in radians.
     */
    void updateRobotPosition(double x, double y, double theta);

    /**
     * @brief Adds a predefined set of static obstacles.
     */
    void setupStaticObstacles();

    /**
     * @brief Checks whether a point is near an obstacle.
     * @param x X coordinate.
     * @param y Y coordinate.
     * @return True if near an obstacle, false otherwise.
     */
    bool isNearObstacle(float x, float y);

    /**
     * @brief Updates robot's current follow point.
     */
    void followCurrentPoint();

    /**
     * @brief Updates the view transform based on pan, rotation, and scale.
     */
    void updateViewTransform();

    /// @}  // end of Support Functions

    /// \name Coordinate Conversion Utilities
    /// @{

    /**
     * @brief Converts world coordinates (meters) to scene coordinates (pixels).
     * @param x_m X in meters.
     * @param y_m Y in meters.
     * @return Corresponding scene coordinates in pixels.
     */
    QPointF worldToScene(double x_m, double y_m);

    /**
     * @brief Converts scene coordinates to map grid coordinates.
     * @param scene_pos Position in scene coordinates.
     * @return Position in map grid coordinates.
     */
    QPointF sceneToMapCoordinates(const QPointF& scene_pos);

    /// @}  // end of Coordinate Conversion Utilities

    /// \name Laser Beam Generation
    /// @{

    /**
     * @brief Creates beam items from current scan data.
     */
    void generateLaserBeams();

    /// @}  // end of Laser Beam Generation

    /// \name Ghost Mode Utilities
    /// @{

    /**
     * @brief Computes a differential drive trajectory for ghost visualization.
     * @param v Linear velocity.
     * @param omega Angular velocity.
     * @param duration_sec Duration in seconds.
     * @param steps Number of steps.
     * @param theta_start_rad Start orientation in radians.
     * @return Vector of Pose2D representing the trajectory.
     */
    std::vector<Pose2D> computeGhostTrajectoryDiffDrive(double v, double omega, double duration_sec,
                                                        int steps, double theta_start_rad);

    /**
     * @brief Starts the ghost animation based on input.
     * @param speed_cm_s Speed in cm/s.
     * @param steering_value Steering value.
     */
    void startGhostAnimation(double speed_cm_s, double steering_value);

    /**
     * @brief Updates ghost animation to current frame.
     * @param x_pos X position.
     * @param y_pos Y position.
     */
    void updateGhostAnimation(double x_pos, double y_pos);

    /**
     * @brief Removes all ghost elements from the scene.
     */
    void deleteGhosts();

    /// @}  // end of Ghost Mode Utilities

    /// \name Trail Mode Utility
    /// @{

    /**
     * @brief Updates trail graphics with the current robot position.
     * @param currentPosition Current robot position in scene coordinates.
     */
    void updateSpeedTrail(const QPointF& currentPosition);

    /// @}  // end of Trail Mode Utility

    /// \name Collision Warning border Mode Utility
    /// @{

    /**
     * @brief Updates the widget's border color based on the minimum laser distance.
     */
    void updateCollisionWarningBorder();

    /// @} // end of Collision Warning border Mode Utility

    /// \name Zone Mode Utility
    /// @{

    /**
     * @brief Removes all zone elements from the scene.
     */
    void deleteZones();

    /// @} // end of Collision Warning border Mode Utility
};

#endif // OBSTACLEMAPWIDGET_H
