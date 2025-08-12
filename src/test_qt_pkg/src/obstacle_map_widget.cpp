#include "obstacle_map_widget.h"

// =====================
// Public Methods
// =====================

/**
 * @brief Constructor for the ObstacleMapWidget class.
 *
 * Initializes the scene and view, sets up default parameters for visualization modes,
 * configures input handling (mouse, gestures), and prepares layout and timers.
 *
 * @param parent The parent widget, if any.
 */
ObstacleMapWidget::ObstacleMapWidget(QWidget* parent)
    : QWidget(parent), m_scene(new QGraphicsScene(this)), m_view(new QGraphicsView(m_scene, this)) {
    // ===== Default parameters for various modes =====
    m_beam_color = Qt::red;
    m_ghost_color = Qt::yellow;
    m_trail_color = Qt::cyan;
    m_laser_number = 540;
    m_curve_gain = 1.25;
    m_ghost_duration = 2.0;

    // ===== Configure view rendering =====
    m_view->setRenderHint(QPainter::Antialiasing);
    m_view->setRenderHint(QPainter::SmoothPixmapTransform);
    m_view->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    // ===== Input handling (mouse + gestures) =====
    m_view->viewport()->installEventFilter(this); // Capture mouse/touch events
    m_view->viewport()->grabGesture(Qt::PinchGesture);
    m_view->viewport()->grabGesture(Qt::PanGesture);
    m_view->viewport()->setAttribute(Qt::WA_AcceptTouchEvents);
    // m_view->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);  // Optional zoom behavior

    // ===== Layout management =====
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(m_view);
    layout->setContentsMargins(0, 0, 0, 0);
    setLayout(layout);

    // ===== Scene initialization =====
    m_scene->setSceneRect(0, 0, 5000, 5000); // Default scene area in pixels
    updateViewTransform();

    // ===== Inertia mode timer setup =====
    m_inertiaTimer.setInterval(30); // Update interval in milliseconds (~33 FPS)
    connect(&m_inertiaTimer, &QTimer::timeout, this, &ObstacleMapWidget::handleInertia);

    // ====== Trail Mode setup ======
    m_trailItem = new QGraphicsPathItem();
    m_scene->addItem(m_trailItem);
}

/**
 * @brief Destructor for ObstacleMapWidget.
 *
 * Qt automatically deletes child widgets and objects with this as parent.
 * Therefore, manual deletion of m_scene and m_view is not necessary and
 * should be avoided to prevent double deletion.
 */
ObstacleMapWidget::~ObstacleMapWidget() {
    // No manual deletion needed; Qt parent-child system manages cleanup.
}

/**
 * @brief Sets the RobotNode instance and initializes the map and robot visualization.
 *
 * If the map is already loaded in the RobotNode, initialization happens immediately.
 * Otherwise, a callback is set to initialize once the map loading completes.
 *
 * @param robot_node Shared pointer to the RobotNode.
 */
void ObstacleMapWidget::setRobotNode(std::shared_ptr<RobotNode> robot_node) {
    m_robot_node = robot_node;

    if (m_robot_node->has_map()) {
        // Map is already loaded, initialize robot immediately via queued connection
        QMetaObject::invokeMethod(this, [this]() { initializeRobot(); }, Qt::QueuedConnection);
    } else {
        // Map not loaded yet, set callback to initialize robot when map is loaded
        m_robot_node->map_loaded = [this]() {
            QMetaObject::invokeMethod(this, [this]() { initializeRobot(); }, Qt::QueuedConnection);
        };
    }
}

/**
 * @brief Sets the navigation node (Nav2Client) used for path planning and control.
 *
 * @param nav2_node Shared pointer to the Nav2Client instance.
 */
void ObstacleMapWidget::setNav2Node(std::shared_ptr<Nav2Client> nav2_node) {
    m_nav2_node = nav2_node;
}

/**
 * @brief stops inertia movement
 */
void ObstacleMapWidget::stopInertia() {
    // Stop robot velocity immediately
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0;
    stop.linear.y = 0;
    stop.angular.z = 0;
    m_robot_node->publish_velocity({stop.linear.x, stop.linear.y}, stop.angular.z);

    // Stop inertia and timer if active
    m_inertiaVelocity = QPointF(0, 0);
    if (m_inertiaTimer.isActive()) {
        m_inertiaTimer.stop();
    }
}

/**
 * @brief Removes all user-drawn elements (paths and points) from the scene.
 *
 * This function clears temporary drawing items such as the currently drawn path
 * and any temporary point markers by removing them from the scene and deleting
 * the corresponding objects.
 */
void ObstacleMapWidget::deleteAllDrawings() {
    // Remove and delete temporary path item if it exists
    if (m_temp_path_item) {
        m_scene->removeItem(m_temp_path_item);
        delete m_temp_path_item;
        m_temp_path_item = nullptr;
    }

    // Remove and delete temporary point item if it exists
    if (m_temp_point_item) {
        m_scene->removeItem(m_temp_point_item);
        delete m_temp_point_item;
        m_temp_point_item = nullptr;
    }
}

/**
 * @brief Callback for handling incoming laser scan messages.
 *
 * This function is called (in RobotNode) whenever new LaserScan data is received.
 * It updates the current scan data and marks it as available for processing.
 *
 * @param msg Shared pointer to the incoming LaserScan message.
 */
void ObstacleMapWidget::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    m_current_scan = *msg;
    m_scan_available = true;
}

/**
 * @brief Getter for GraphicsView Information
 *
 */
ObstacleMapWidget::ViewData ObstacleMapWidget::getViewData() const {
    ViewData view;
    view.pos = m_view->mapToScene(m_view->viewport()->rect().center());
    view.rot = m_currentRotation_view;
    view.zoom = m_currentScale_view;
    return view;
}

// =====================
// Protected Methods
// =====================

/**
 * @brief Handles widget resize events.
 *
 * Adjusts the QGraphicsView to fit the scene while maintaining aspect ratio,
 * and centers the view on the robot's current pixel position.
 *
 * @param event Pointer to the resize event.
 */
void ObstacleMapWidget::resizeEvent(QResizeEvent* event) {
    // Call base class implementation to ensure default behavior
    QWidget::resizeEvent(event);

    // Fit the scene into the view, preserving aspect ratio
    // m_view->fitInView(m_scene->sceneRect(), Qt::KeepAspectRatio);

    // Center the view on the robot's current position in pixel coordinates
    m_view->centerOn(m_robot_x_pixels, m_robot_y_pixels);
}

/**
 * @brief Event filter to handle mouse, touch, and gesture events on the viewport.
 *
 * Interprets user interactions for drawing paths, following points, inertia mode,
 * and supports gestures like pinch zoom and panning.
 *
 * @param obj Pointer to the QObject receiving the event (usually viewport).
 * @param event Pointer to the event being processed.
 * @return true if the event was handled and should not propagate further,
 *         false to continue normal event processing.
 */
bool ObstacleMapWidget::eventFilter(QObject* obj, QEvent* event) {
    if (obj == m_view->viewport()) {
        // =====================
        // Mouse Button Pressed
        // =====================
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);

            if (m_drawPathMode) {
                // Clear any temporary drawings from previous interactions
                deleteAllDrawings();

                // Start drawing a new path
                m_drawing = true;
                m_path_points.clear();
                QPointF scenePos = m_view->mapToScene(mouseEvent->pos());
                m_path_points.push_back(scenePos);
            } else if (m_followMode) {
                // Start following a point
                m_following = true;
                m_current_follow_point = m_view->mapToScene(mouseEvent->pos());
            } else if (m_inertiaMode) {
                // Initialize inertia interaction (for flick gesture)
                m_inertiaTimer.stop();
                m_inertiaStart = m_view->mapToScene(mouseEvent->pos());
                m_inertiaStartTime = QTime::currentTime();
            }

            return true; // Event handled
        }

        // =====================
        // Mouse Move
        // =====================
        else if (event->type() == QEvent::MouseMove) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);

            if (m_drawPathMode) {
                if (!m_drawing)
                    return false; // Ignore if not currently drawing

                // Add new point to path and redraw temporary path
                QPointF scenePos = m_view->mapToScene(mouseEvent->pos());
                m_path_points.push_back(scenePos);

                QPainterPath path;
                path.moveTo(m_path_points.first());
                for (const auto& pt : m_path_points)
                    path.lineTo(pt);

                // Remove previous temp path if exists
                if (m_temp_path_item) {
                    m_scene->removeItem(m_temp_path_item);
                    delete m_temp_path_item;
                }

                // Draw new temp path in blue
                m_temp_path_item = m_scene->addPath(path, QPen(Qt::blue, 2));
            } else if (m_followMode) {
                // Update follow point while moving mouse
                m_current_follow_point = m_view->mapToScene(mouseEvent->pos());
            }

            return true; // Event handled
        }

        // =====================
        // Mouse Button Released
        // =====================
        else if (event->type() == QEvent::MouseButtonRelease) {
            if (m_drawPathMode) {
                m_drawing = false;

                if (m_path_points.size() >= 2) {
                    // Path complete, emit signal or process path
                    pathDrawn(m_path_points);
                } else if (m_path_points.size() == 1) {
                    // Single point — draw a small circle as visual feedback
                    QPointF singlePoint = m_path_points.front();
                    constexpr qreal radius = 3.0;

                    m_temp_point_item = m_scene->addEllipse(
                        singlePoint.x() - radius, singlePoint.y() - radius, 2 * radius, 2 * radius,
                        QPen(Qt::blue), QBrush(Qt::blue));

                    pathDrawn(m_path_points);
                }

                return true; // Event handled
            } else if (m_followMode) {
                m_following = false;

                // Cancel any active navigation goals
                m_nav2_node->cancelGoalsPose();
                return true; // Event handled
            } else if (m_inertiaMode) {
                // Calculate flick velocity for inertia scrolling
                QPointF end = m_view->mapToScene(static_cast<QMouseEvent*>(event)->pos());
                int elapsed_ms = m_inertiaStartTime.msecsTo(QTime::currentTime());

                if (elapsed_ms > 0) {
                    double dt = elapsed_ms / 1000.0;
                    QPointF delta_pixels =
                        QPointF(end.x() - m_inertiaStart.x(), m_inertiaStart.y() - end.y());
                    QPointF delta_meters = delta_pixels / m_pixels_per_meter;

                    QPointF velocity_mps = delta_meters / dt;

                    constexpr double damping = 0.01;
                    velocity_mps *= damping;

                    m_inertiaVelocity = velocity_mps;
                    m_inertiaTimer.start(); // Start periodic inertia handling
                }

                return true; // Event handled
            }
        }

        // =====================
        // Touch Events (Begin, Update, End)
        // =====================
        if (event->type() == QEvent::TouchBegin || event->type() == QEvent::TouchUpdate ||
            event->type() == QEvent::TouchEnd) {

            QTouchEvent* touchEvent = static_cast<QTouchEvent*>(event);
            const auto& touchPoints = touchEvent->touchPoints();
            m_current_finger_count = touchPoints.count();

            // === 1-Finger Touch Panning (when no special mode is active) ===
            if (!m_zoneMode && !m_drawPathMode && !m_inertiaMode && !m_followMode) {
                if (touchPoints.count() == 1) {
                    const auto& point = touchPoints.first();

                    if (event->type() == QEvent::TouchBegin) {
                        // Save the initial touch position
                        m_lastTouchPos = point.pos();
                        m_touchPanningActive = true;
                        return true;
                    }

                    if (event->type() == QEvent::TouchUpdate && m_touchPanningActive) {
                        QPointF currentPos = touchPoints.first().pos();
                        QPointF delta = currentPos - m_lastTouchPos;

                        auto* hBar = m_view->horizontalScrollBar();
                        auto* vBar = m_view->verticalScrollBar();

                        hBar->setValue(hBar->value() - delta.x());
                        vBar->setValue(vBar->value() - delta.y());

                        m_lastTouchPos = currentPos;
                        return true;
                    }

                    if (event->type() == QEvent::TouchEnd) {
                        // End panning
                        m_touchPanningActive = false;
                        return true;
                    }
                } else {
                    const auto& point = touchPoints.first();
                    m_lastTouchPos = point.pos();
                }
            }

            // ===== Zone Mode =====
            if (m_zoneMode) {

                if (m_activeFingerCount < touchPoints.size())
                    m_activeFingerCount = touchPoints.size();

                // Start Zone
                if (event->type() == QEvent::TouchBegin) {
                    if (touchPoints.size() == 0)
                        return true;

                    m_zoneDrawingInProgress = true;
                    m_touchStartCenter = m_view->mapToScene(touchPoints[0].pos().toPoint());

                    // Delete current and saved zones
                    deleteZones();

                    // Setup active zone
                    m_activeZone.center = m_touchStartCenter;
                    m_activeZone.radius = 0;
                    m_activeZone.corners = std::min(m_activeFingerCount, 10);
                    m_activeZone.valid = true;

                    // Initialize rotation
                    if (touchPoints.size() >= 2) {
                        QPointF p1 = m_view->mapToScene(touchPoints[0].pos().toPoint());
                        QPointF p2 = m_view->mapToScene(touchPoints[1].pos().toPoint());
                        QLineF line(p1, p2);
                        m_initialRotationAngle_zone = line.angle();
                        m_currentRotationAngle_zone = 0;
                    }

                    // Create new Zone (polygon item)
                    QPolygonF poly = m_activeZone.polygon();
                    m_activeZone.graphicsItem = m_scene->addPolygon(poly, QPen(Qt::green, 2),
                                                                    QBrush(QColor(0, 255, 0, 50)));

                    return true;
                }
                // Update Zone
                if (event->type() == QEvent::TouchUpdate && m_zoneDrawingInProgress) {

                    // Calculate radius from biggest touch distance
                    qreal maxDistance = 0;
                    for (const auto& pt : touchPoints) {
                        QPointF scenePt = m_view->mapToScene(pt.pos().toPoint());
                        qreal dist = QLineF(m_touchStartCenter, scenePt).length();
                        if (dist > maxDistance)
                            maxDistance = dist;
                    }

                    // Update radius and polygon corners
                    m_activeZone.radius = maxDistance;
                    m_activeZone.corners = std::min(m_activeFingerCount, 10);

                    // Update rotation
                    if (touchPoints.size() >= 2) {
                        QPointF p1 = m_view->mapToScene(touchPoints[0].pos().toPoint());
                        QPointF p2 = m_view->mapToScene(touchPoints[1].pos().toPoint());
                        QLineF line(p1, p2);
                        qreal newAngle = line.angle();
                        qreal deltaAngle = newAngle - m_initialRotationAngle_zone;

                        // Normiere Delta-Winkel
                        if (deltaAngle > 180)
                            deltaAngle -= 360;
                        if (deltaAngle < -180)
                            deltaAngle += 360;

                        m_currentRotationAngle_zone = deltaAngle;
                    }

                    m_activeZone.rotation = m_currentRotationAngle_zone;

                    // Update polygon
                    if (m_activeZone.graphicsItem) {
                        QPolygonF poly = m_activeZone.polygon(m_activeZone.rotation);
                        m_activeZone.graphicsItem->setPolygon(poly);
                    }

                    return true;
                }
                // End Zone
                if (event->type() == QEvent::TouchEnd && m_zoneDrawingInProgress) {
                    m_zoneDrawingInProgress = false;

                    if (m_activeZone.valid) {
                        // Save zone
                        m_savedZones.append(m_activeZone);

                        // Start moving to center of polygon (Here a good zone-search-algorithm can
                        // be implemented)
                        m_current_path.clear();
                        m_current_path.append(m_activeZone.center);
                        m_current_target_index = 0;
                        goToNextPoint();

                        // reset active zone, so new zones can spawn
                        m_activeZone.graphicsItem = nullptr;
                        m_activeZone.valid = false;
                        m_activeFingerCount = 0;
                    }

                    return true;
                }
            } else {
                deleteZones();
            }

            // ===== Intertia Mode =====
            if (m_inertiaMode && !m_zoneMode && touchPoints.count() >= 2) {
                // Multiple fingers detected — stop inertia and robot movement

                qDebug() << "Multiple fingers detected – stopping inertia";

                // Stop robot velocity immediately
                geometry_msgs::msg::Twist stop;
                stop.linear.x = 0;
                stop.linear.y = 0;
                stop.angular.z = 0;
                m_robot_node->publish_velocity({stop.linear.x, stop.linear.y}, stop.angular.z);

                // Stop inertia and timer if active
                m_inertiaVelocity = QPointF(0, 0);
                if (m_inertiaTimer.isActive()) {
                    m_inertiaTimer.stop();
                }

                return true; // Event handled
            }
        }

        // =====================
        // Gesture Events (Pinch Zoom and Pan)
        // =====================
        if (event->type() == QEvent::Gesture) {
            QGestureEvent* gestureEvent = static_cast<QGestureEvent*>(event);

            // Handle pinch gesture for zoom and rotation
            if (QGesture* g = gestureEvent->gesture(Qt::PinchGesture)) {
                if (!m_zoneMode && !m_drawPathMode && !m_inertiaMode && !m_followMode) {
                    QPinchGesture* pinch = static_cast<QPinchGesture*>(g);

                    if (pinch->state() == Qt::GestureStarted) {
                        // Remember start rotation
                        m_startPinchRotation = pinch->rotationAngle();

                        m_initialScale_view = m_currentScale_view;
                        m_initialRotation_view = m_currentRotation_view;
                    }

                    if (pinch->state() == Qt::GestureStarted ||
                        pinch->state() == Qt::GestureUpdated) {

                        // Relative rotation (always start with 0 rotation)
                        qreal relativeRotation = pinch->rotationAngle() - m_startPinchRotation;

                        // Absolute scale and rotation based on initial values
                        m_currentScale_view = m_initialScale_view * pinch->totalScaleFactor();
                        m_currentRotation_view = m_initialRotation_view + relativeRotation;

                        updateViewTransform();
                        return true;
                    }

                    if (pinch->state() == Qt::GestureFinished ||
                        pinch->state() == Qt::GestureCanceled) {
                        return true;
                    }
                }
            }
        }
    }

    // Forward unhandled events to base class eventFilter
    return QWidget::eventFilter(obj, event);
}

// =====================
// Private Slot Methods
// =====================

/**
 * @brief Slot triggered periodically to handle inertia simulation.
 *
 * Connected to a QTimer, this function updates the robot's velocity based on
 * the current inertia velocity, applying friction to gradually reduce speed,
 * and stops the motion when the velocity is sufficiently small.
 */
void ObstacleMapWidget::handleInertia() {
    constexpr double max_speed = 0.4; // Maximum allowed speed (m/s)
    constexpr double friction = 0.99; // Friction factor to reduce velocity each step

    // Stop inertia if velocity is very small or inertia mode is disabled
    if ((std::abs(m_inertiaVelocity.x()) < 0.01 && std::abs(m_inertiaVelocity.y()) < 0.01) ||
        !m_inertiaMode) {
        m_inertiaTimer.stop();

        // Optionally stop the robot completely by publishing zero velocity
        m_robot_node->publish_velocity({0, 0}, m_robot_node->getRotationNormalized());
        return;
    }

    // Clamp velocity components to the maximum speed limits
    double vx_robot = std::clamp(m_inertiaVelocity.x(), -max_speed, max_speed);
    double vy_robot = std::clamp(m_inertiaVelocity.y(), -max_speed, max_speed);

    // Normalize velocity to range [-1, 1] for direct control input
    double norm_x = vx_robot / max_speed;
    double norm_y = vy_robot / max_speed;

    // Prepare velocity message to send to robot
    geometry_msgs::msg::Vector3 velocity_msg;
    velocity_msg.x = std::clamp(norm_x, -1.0, 1.0);
    velocity_msg.y = std::clamp(norm_y, -1.0, 1.0);

    // Publish the current velocity to the robot with the current rotation
    m_robot_node->publish_velocity({velocity_msg.x, velocity_msg.y},
                                   m_robot_node->getRotationNormalized());

    // Apply friction to gradually slow down the velocity for inertia effect
    m_inertiaVelocity *= friction;
}

// =====================
// Private Methods
// =====================

/**
 * @brief Prepares robot and map visualization once the map is available.
 *
 * Sets the AMCL pose callback to update robot position and orientation.
 * Initializes timers to update robot pose visualization (every 10 ms),
 * generate laser beam visualization (every 100 ms), and update follow mode
 * targets (every 1000 ms). Also draws the robot ellipse and orientation line
 * and centers the view on the scene.
 *
 * Registers this widget with the navigation node for obstacle map access.
 */
void ObstacleMapWidget::initializeRobot() {
    // Set callback to process AMCL pose updates
    m_robot_node->on_amcl_pose_received =
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            // Extract quaternion from message
            tf2::Quaternion tf_q;
            tf2::fromMsg(msg->pose.pose.orientation, tf_q);

            double roll, pitch, yaw;
            tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

            // Update robot position in meters
            m_robot_x_meters = msg->pose.pose.position.x;
            m_robot_y_meters = msg->pose.pose.position.y;

            // Update robot orientation (yaw in radians)
            m_robot_theta_rad = yaw;

            has_amcl = true;
        };

    // Timer to update robot visualization every 10 ms using latest AMCL pose
    QTimer* update_pose_timer = new QTimer(this);
    connect(update_pose_timer, &QTimer::timeout, this, [this]() {
        updateRobotPosition(m_robot_x_meters, m_robot_y_meters, m_robot_theta_rad);
        if (has_amcl) {
            if (m_follow_robot_on_map) {
                m_view->centerOn(m_robot_x_pixels, m_robot_y_pixels);
            }

            if (!m_mapFollow) {
                m_follow_robot_on_map = false;
            } else {
                m_follow_robot_on_map = true;
            }
        }
    });
    update_pose_timer->start(10);

    // Draw robot as ellipse on the scene
    m_robot_ellipse = m_scene->addEllipse(m_robot_x_pixels - m_robot_size / 2,
                                          m_robot_y_pixels - m_robot_size / 2, m_robot_size,
                                          m_robot_size, QPen(Qt::green), QBrush(Qt::green));

    // Draw orientation line indicating robot's heading
    double end_x = m_robot_x_pixels + m_robot_size * cos(m_robot_theta_rad);
    double end_y = m_robot_y_pixels - m_robot_size * sin(m_robot_theta_rad);

    m_orientation_line =
        m_scene->addLine(m_robot_x_pixels, m_robot_y_pixels, end_x, end_y, QPen(Qt::blue, 2));

    // Center view on the entire scene (map)
    // m_view->fitInView(m_scene->sceneRect(), Qt::KeepAspectRatio);

    // Update obstacle visualization on the map
    updateObstacles();

    // Register this widget with the navigation node to provide obstacle map access
    m_nav2_node->setObstacleMap(this);

    // Set finish Callback function of nav2 Client, so all drawings will be deleted after path is
    // traversed This is needed, so the function will be executed on the Qt-Thread
    m_nav2_node->setOnPathFinishedCallback(
        [this]() { QMetaObject::invokeMethod(this, "deleteAllDrawings", Qt::QueuedConnection); });

    // Timer to generate laser beam visualization every 100 ms
    QTimer* beam_timer = new QTimer(this);
    connect(beam_timer, &QTimer::timeout, this, &ObstacleMapWidget::generateLaserBeams);
    beam_timer->start(100);

    // Timer to update follow mode goal every 1000 ms
    QTimer* follow_timer = new QTimer(this);
    connect(follow_timer, &QTimer::timeout, this, &ObstacleMapWidget::followCurrentPoint);
    follow_timer->start(1000);

    // Timer to update collision border warning every 100 ms
    QTimer* collision_border_timer = new QTimer(this);
    connect(collision_border_timer, &QTimer::timeout, this,
            &ObstacleMapWidget::updateCollisionWarningBorder);
    collision_border_timer->start(100);
}

/**
 * @brief Sends the robot to the next point in the current path.
 *
 * This function handles navigation commands based on the size of the current path:
 * - If the path contains only one point, it sends a single goal pose to the navigation client.
 * - If the path contains multiple points, it constructs a full path message and sends it to the
 * navigation client.
 *
 * If the current target index exceeds the path size, the robot is commanded to stop.
 */
void ObstacleMapWidget::goToNextPoint() {
    // Stop the robot if all points are processed
    if (m_current_target_index >= m_current_path.size()) {
        // Stop robot
        m_robot_node->publish_velocity({0.0, 0.0}, 0.0);

        // Cleanup or reset internal state here
        m_current_path.clear();
        m_current_target_index = 0;

        qDebug() << "Navigation finished, cleaned up.";

        return;
    }

    // Single pose client (only one target point)
    if (m_current_path.size() == 1) {
        QPointF target = m_current_path[m_current_target_index];
        QPointF target_world = sceneToMapCoordinates(target);

        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = m_nav2_node->now();

        goal.pose.position.x = target_world.x();
        goal.pose.position.y = target_world.y();
        goal.pose.position.z = 0.0;
        // goal.pose.orientation.w = 1.0; // no rotation

        // Send the single goal to the navigation client
        m_nav2_node->sendGoal(goal);
    }
    // Path following client
    else {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = m_nav2_node->now();

        // Convert each point in the current path from scene to world coordinates and append to
        // path
        for (const QPointF& pt : m_current_path) {
            QPointF world = sceneToMapCoordinates(pt);

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = world.x();
            pose.pose.position.y = world.y();
            pose.pose.position.z = 0.0;
            // pose.pose.orientation.w = 1.0;  // no rotation
            path.poses.push_back(pose);
        }

        // Send the full path to the navigation client
        m_nav2_node->sendPath(path);
    }
}

/**
 * @brief Handles the event when a new path is drawn by the user.
 *
 * This function resamples the input path points to have a fixed spacing,
 * resets the current target index, and initiates navigation to the first point.
 *
 * @param points The list of points defining the drawn path in scene coordinates.
 */
void ObstacleMapWidget::pathDrawn(const QVector<QPointF>& points) {
    // Resample path points to have consistent spacing (e.g., 5.0 units)
    m_current_path = resamplePath(points, 5.0);

    // Reset target index to start navigating from the first point
    m_current_target_index = 0;

    // Start navigation towards the next target point
    goToNextPoint();
}

/**
 * @brief Renders the obstacle visualization based on the current occupancy grid map.
 *
 * This function clears the current visualization and generates a new image
 * representing occupied cells from the occupancy grid map as black pixels.
 * The image is converted into a single QGraphicsPixmapItem and added to the scene
 * for efficient rendering. The scene rectangle and view are adjusted accordingly.
 */
void ObstacleMapWidget::updateObstaclesFromMap() {
    if (!m_robot_node->has_map())
        return;

    // Load occupancy grid map
    m_map = m_robot_node->get_map();
    int width = m_map.info.width;
    int height = m_map.info.height;
    double resolution = m_map.info.resolution;
    double cellSize = resolution * m_pixels_per_meter;

    // Map origin in world coordinates (meters)
    double origin_x = m_map.info.origin.position.x;
    double origin_y = m_map.info.origin.position.y;

    // Create an image representing the map (1 pixel per map cell)
    QImage image(width, height, QImage::Format_ARGB32);
    image.fill(Qt::transparent); // Fill with transparent background

    // Mark occupied cells (value 100) as black pixels
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int i = x + y * width;
            if (m_map.data[i] >= 0.65) {
                image.setPixel(x, y, qRgba(0, 0, 0, 255)); // Black pixel
            }
        }
    }

    // Mirror vertically to match Qt's coordinate system (Y-axis downwards)
    image = image.mirrored(false, true);

    // Convert to pixmap and scale to physical size in pixels
    QPixmap pixmap = QPixmap::fromImage(image);

    // Convert map origin to scene coordinates
    double scene_left = origin_x * m_pixels_per_meter;
    double scene_top = -origin_y * m_pixels_per_meter - height * cellSize;

    // Add the pixmap as a single item to the scene
    auto* pixmapItem = new QGraphicsPixmapItem(pixmap);
    pixmapItem->setPos(scene_left, scene_top);
    pixmapItem->setScale(cellSize);
    m_scene->addItem(pixmapItem);

    // Define the visible area of the scene with some margin
    constexpr int margin = 500;
    m_scene->setSceneRect(scene_left - margin, scene_top - margin, width * cellSize + 2 * margin,
                          height * cellSize + 2 * margin);

    updateViewTransform();

    // Adjust the view to fit the scene and center on the robot position
    // m_view->fitInView(QRectF(scene_left, scene_top, width * cellSize, height * cellSize),
    //                   Qt::KeepAspectRatio);
    m_view->centerOn(m_robot_x_pixels, m_robot_y_pixels);
}

/**
 * @brief Resamples a path to produce points spaced evenly by a given distance.
 *
 * This function takes an input path (a vector of QPointF) and returns a new path
 * with points spaced approximately `spacing` units apart by linear interpolation.
 * If the input path has fewer than two points, it is returned unchanged.
 *
 * @param originalPoints The original path as a vector of points.
 * @param spacing The desired distance between consecutive points in the resampled path
 * (default 5.0).
 * @return QVector<QPointF> The resampled path with evenly spaced points.
 */
QVector<QPointF> ObstacleMapWidget::resamplePath(const QVector<QPointF>& originalPoints,
                                                 double spacing) {
    if (originalPoints.size() < 2)
        return originalPoints;

    QVector<QPointF> newPoints;
    newPoints.push_back(originalPoints.first());

    double accumulatedDist = 0.0;
    for (int i = 1; i < originalPoints.size(); ++i) {
        QPointF p0 = originalPoints[i - 1];
        QPointF p1 = originalPoints[i];
        double segmentLength = QLineF(p0, p1).length();

        // Interpolate points along segment while spacing is reached or exceeded
        while (accumulatedDist + segmentLength >= spacing) {
            double t = (spacing - accumulatedDist) / segmentLength;
            QPointF newPoint = p0 + t * (p1 - p0);
            newPoints.push_back(newPoint);

            // Update segment start point and length for next interpolation
            p0 = newPoint;
            segmentLength = QLineF(p0, p1).length();
            accumulatedDist = 0;
        }
        accumulatedDist += segmentLength;
    }

    // Append last point if not already included
    if (!newPoints.last().isNull() && newPoints.last() != originalPoints.last())
        newPoints.push_back(originalPoints.last());

    return newPoints;
}

/**
 * @brief Adds a rectangular obstacle to the scene.
 *
 * Creates a red rectangle representing an obstacle at the specified position and size,
 * and adds it to the graphics scene for visualization.
 *
 * @param x The x-coordinate of the obstacle's top-left corner in scene coordinates.
 * @param y The y-coordinate of the obstacle's top-left corner in scene coordinates.Ja ne d
 * @param width The width of the obstacle rectangle.
 * @param height The height of the obstacle rectangle.
 */
void ObstacleMapWidget::addObstacle(double x, double y, double width, double height) {
    QGraphicsRectItem* rect = new QGraphicsRectItem(x, y, width, height);
    rect->setBrush(QBrush(Qt::red)); // Obstacle represented as red rectangle
    m_scene->addItem(rect);
}

/**
 * @brief Updates the obstacle visualization on the map.
 *
 * This function refreshes the displayed obstacles by
 * reloading and drawing them from the current occupancy map.
 * It currently delegates the work to updateObstaclesFromMap().
 */
void ObstacleMapWidget::updateObstacles() {
    updateObstaclesFromMap();

    // ===== Optional: Add test/static obstacles for debugging =====
    setupStaticObstacles();
}

/**
 * @brief Updates the robot's position and orientation in the visualization.
 *
 * Converts the given world coordinates (x, y in meters) to scene coordinates (pixels),
 * updates the graphical representation of the robot (ellipse and orientation line),
 * and handles additional visualization modes such as ghost and trail modes.
 *
 * @param x Robot's x-position in world coordinates (meters).
 * @param y Robot's y-position in world coordinates (meters).
 * @param theta Robot's orientation (yaw) in radians.
 */
void ObstacleMapWidget::updateRobotPosition(double x, double y, double theta) {
    if (!m_robot_node->has_map())
        return;

    QPointF robo_pos = worldToScene(x, y);
    m_robot_x_pixels = robo_pos.x();
    m_robot_y_pixels = robo_pos.y();
    m_robot_theta_rad = theta;

    if (m_robot_ellipse) {
        // Draw robot as ellipse centered at robot position with rotation
        m_robot_ellipse->setZValue(2);
        m_robot_ellipse->setRect(0, 0, m_robot_size, m_robot_size);
        m_robot_ellipse->setPos(m_robot_x_pixels - m_robot_size / 2,
                                m_robot_y_pixels - m_robot_size / 2);
        m_robot_ellipse->setTransformOriginPoint(m_robot_size / 2, m_robot_size / 2);
        m_robot_ellipse->setRotation(-m_robot_theta_rad * 180.0 / M_PI);
    }

    if (m_orientation_line) {
        // Update orientation line showing robot heading
        double length = m_robot_size / 2;
        double endX = m_robot_x_pixels + length * std::cos(m_robot_theta_rad);
        double endY = m_robot_y_pixels - length * std::sin(m_robot_theta_rad);
        m_orientation_line->setLine(m_robot_x_pixels, m_robot_y_pixels, endX, endY);
        m_orientation_line->setZValue(3); // Draw on top
    }

    // Update ghost animation if enabled and speed or steering changed significantly
    if (m_ghostMode) {
        double current_speed = m_robot_node->getSpeed().x * 100.0; // convert to cm/s
        double current_steering = m_robot_node->getRotation();

        if (std::abs(m_last_speed - current_speed) > 2 ||
            std::abs(m_last_steering - current_steering) > 1e-1) {
            startGhostAnimation(current_speed, -current_steering);
            m_last_speed = current_speed;
            m_last_steering = current_steering;
        }
    } else {
        deleteGhosts();
    }

    // Update trail visualization if enabled
    if (m_trailMode) {
        QPointF scenePos = worldToScene(x, y);
        updateSpeedTrail(scenePos);
    }
}

/**
 * @brief Sets up the static obstacles on the map as pairs of rotated stripes.
 *
 * This function generates multiple obstacle pairs (like narrow gates) made up of
 * two parallel, rotated rectangles. Each pair is placed at a random position on the map
 * with a random rotation, forcing the tester to navigate between the stripes.
 */
void ObstacleMapWidget::setupStaticObstacles() {
    const int numPairs = 5;
    const int stripWidth = 5;
    const int stripHeight = 150;
    const int gapBetweenStrips = 200;
    double resolution = m_map.info.resolution;
    double cellSize = resolution * m_pixels_per_meter;

    // Predefined positions (center points between stripes)
    struct ObstaclePair {
        int cx;
        int cy;
        float angleDeg;
    };

    std::array<ObstaclePair, numPairs> positions = {{{500, -300, 0.0f},
                                                     {400, -250, 45.0f},
                                                     {800, -400, 90.0f},
                                                     {700, -100, 135.0f},
                                                     {550, -150, 270.0f}}};

    // Predefined colors (1 color per pair)
    std::array<QColor, numPairs> colors = {{Qt::red, Qt::green, Qt::blue, Qt::yellow, Qt::magenta}};

    for (size_t i = 0; i < positions.size(); ++i) {
        const auto& pair = positions[i];
        const QColor& color = colors[i];

        float angleRad = qDegreesToRadians(pair.angleDeg);

        // Offset along the rotation direction
        float dx = (gapBetweenStrips / 2.0f) * qCos(angleRad);
        float dy = (gapBetweenStrips / 2.0f) * qSin(angleRad);

        float x1 = pair.cx * cellSize - dx;
        float y1 = pair.cy * cellSize - dy;
        float x2 = pair.cx * cellSize + dx;
        float y2 = pair.cy * cellSize + dy;

        addRotatedObstacle(x1, y1, stripWidth, stripHeight, pair.angleDeg, color);
        addRotatedObstacle(x2, y2, stripWidth, stripHeight, pair.angleDeg, color);
    }
}

/**
 * @brief Adds a rotated rectangular obstacle to the map.
 *
 * This function creates a rectangle centered at the specified (x, y) position,
 * rotated by the given angle. The rectangle is added to the scene and rendered
 * as a red obstacle.
 *
 * @param x The x-coordinate of the center of the obstacle.
 * @param y The y-coordinate of the center of the obstacle.
 * @param width The width of the obstacle.
 * @param height The height of the obstacle.
 * @param angleDeg The rotation angle in degrees (clockwise).
 */
void ObstacleMapWidget::addRotatedObstacle(float x, float y, float width, float height,
                                           float angleDeg, QColor color) {
    // Create a rectangle centered at (0, 0)
    QGraphicsRectItem* rect = new QGraphicsRectItem(-width / 2.0, -height / 2.0, width, height);
    rect->setBrush(QBrush(color)); // Represent the obstacle with a red fill

    // Move and rotate the item
    rect->setPos(x, y);
    rect->setRotation(angleDeg);

    // Add it to the scene
    m_scene->addItem(rect);
}

/**
 * @brief Checks if the given point is near any obstacle in the scene.
 *
 * The robot is modeled as a circle with a fixed radius, and the function calculates
 * the minimum distance between this circle centered at (x, y) and all rectangular obstacles.
 * If the distance is less than the sum of the robot radius and a safety margin
 * (EMERGENCY_STOP_RADIUS), it indicates a potential collision risk.
 *
 * @param x X coordinate of the robot center in scene coordinates.
 * @param y Y coordinate of the robot center in scene coordinates.
 * @return true if the point is too close to an obstacle (emergency stop required), false
 * otherwise.
 */
bool ObstacleMapWidget::isNearObstacle(float x, float y) {
    // Robot modeled as circle with center (x,y) and fixed radius
    const float robotRadius = 10.0f;

    for (auto item : m_scene->items()) {
        auto rectItem = dynamic_cast<QGraphicsRectItem*>(item);
        if (!rectItem)
            continue;

        QRectF rect = rectItem->rect();

        // Calculate minimum distance between circle center and rectangle edges
        float closestX = std::max(float(rect.left()), std::min(x, float(rect.right())));
        float closestY = std::max(float(rect.top()), std::min(y, float(rect.bottom())));

        float dx = x - closestX;
        float dy = y - closestY;

        float distance = std::sqrt(dx * dx + dy * dy);

        if (distance < (robotRadius + EMERGENCY_STOP_RADIUS)) {
            return true; // Emergency stop needed
        }
    }

    return false;
}

/**
 * @brief Sends the current finger or mouse position as a navigation goal in follow mode.
 *
 * This function is called periodically (every ... ms) to update the robot's goal
 * position to follow the user's current touch or mouse location on the map.
 * If follow mode is inactive or not currently following, no command is sent.
 */
void ObstacleMapWidget::followCurrentPoint() {
    // Robot does not move if follow mode is off or no current follow point
    if (!m_followMode || !m_following) {
        return;
    }

    QPointF world = sceneToMapCoordinates(m_current_follow_point);
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = m_nav2_node->now();
    goal.header.frame_id = "map";

    goal.pose.position.x = world.x();
    goal.pose.position.y = world.y();
    goal.pose.position.z = 0.0;

    // Send the goal to the navigation client
    m_nav2_node->sendGoal(goal);
}

/**
 * @brief Applies pan, rotation, and zoom to the view transform.
 *
 * Updates the QGraphicsView transform using current values for panning,
 * rotation (in degrees), and scaling.
 */
void ObstacleMapWidget::updateViewTransform() {
    QTransform t;
    // t.translate(-m_panOffset_view.x(), -m_panOffset_view.y());
    t.rotate(m_currentRotation_view);
    t.scale(m_currentScale_view, m_currentScale_view);
    m_view->setTransform(t);
}

/**
 * @brief Converts world coordinates (meters) to Qt scene pixel coordinates.
 *
 * The Y-axis is inverted because Qt's coordinate system grows downward,
 * whereas the world coordinate system grows upward.
 *
 * @param x_m X position in meters (world coordinates)
 * @param y_m Y position in meters (world coordinates)
 * @return QPointF Corresponding position in Qt scene pixels
 */
QPointF ObstacleMapWidget::worldToScene(double x_m, double y_m) {
    double scene_x = x_m * m_pixels_per_meter;
    double scene_y = -y_m * m_pixels_per_meter;

    // qDebug() << "worldToScene: x_m=" << x_m << " y_m=" << y_m
    //          << " -> scene_x=" << scene_x << " scene_y=" << scene_y;

    return QPointF(scene_x, scene_y);
}

/**
 * @brief Converts Qt scene pixel coordinates to world coordinates in meters.
 *
 * The Y-axis is inverted because Qt's coordinate system grows downward,
 * whereas the world coordinate system grows upward.
 *
 * @param scene_pos Position in Qt scene pixels
 * @return QPointF Corresponding position in world coordinates (meters)
 */
QPointF ObstacleMapWidget::sceneToMapCoordinates(const QPointF& scene_pos) {
    double x_m = scene_pos.x() / m_pixels_per_meter;
    double y_m = -scene_pos.y() / m_pixels_per_meter;

    return QPointF(x_m, y_m);
}

/**
 * @brief Generates and visualizes laser beams in the scene based on the latest LIDAR scan data.
 *
 * When beam mode is enabled and scan data is available, this function clears previously drawn
 * beams and draws new laser rays as lines originating from the robot's current position. Each
 * beam corresponds to a LIDAR range measurement, transformed to scene coordinates.
 *
 * If beam mode is disabled or no scan data is available, all existing beams are removed from
 * the scene.
 *
 * The beams visually represent obstacles detected by the LIDAR sensor.
 */
void ObstacleMapWidget::generateLaserBeams() {
    if (m_scan_available) {

        // For minimal and average laser distance
        int filter = 30;
        float dist_sum = 0;

        // Clear previous beams
        for (auto item : m_beam_items) {
            m_scene->removeItem(item);
            delete item;
        }
        m_beam_items.clear();

        int numRays = m_current_scan.ranges.size();
        int count = std::min(m_laser_number, numRays); // Limit to available rays

        // Saving min distance for collision warnings
        m_min_laser_distance = std::numeric_limits<float>::max();

        for (int i = 0; i < count; ++i) {
            int index = static_cast<int>((i / static_cast<float>(count - 1)) * (numRays - 1));
            float angle = m_current_scan.angle_min + index * m_current_scan.angle_increment;
            float dist = m_current_scan.ranges[index];
            if (!std::isfinite(dist))
                continue;

            // Get current min_distance for collision warning modes
            // Filter out the widest lasers
            if (i < (numRays - filter) && i > filter) {
                m_min_laser_distance = std::min(m_min_laser_distance, dist);
                dist_sum += dist;
            }

            float theta = angle + m_robot_theta_rad;

            float endX = m_robot_x_pixels + dist * m_pixels_per_meter * std::cos(theta);
            float endY = m_robot_y_pixels - dist * m_pixels_per_meter * std::sin(theta);

            if (m_beamMode) {
                float offset_meters = 0.3f;
                float offset_pixels = offset_meters * m_pixels_per_meter;

                float startX = m_robot_x_pixels + offset_pixels * std::cos(m_robot_theta_rad);
                float startY = m_robot_y_pixels - offset_pixels * std::sin(m_robot_theta_rad);

                QGraphicsLineItem* line =
                    m_scene->addLine(startX, startY, endX, endY, QPen(m_beam_color));
                line->setZValue(0);
                m_beam_items.push_back(line);
            }
        }

        m_avg_laser_distance = dist_sum / (numRays - 2 * filter);

    } else {
        // Remove all beams if beam mode is off or no scan data
        for (auto item : m_beam_items) {
            m_scene->removeItem(item);
            delete item;
        }
        m_beam_items.clear();
    }
}

/**
 * @brief Computes a simulated trajectory (ghost path) of a differential drive robot.
 *
 * This function calculates the predicted poses of a robot moving with a given
 * forward velocity (v) and angular velocity (omega) over a specified duration,
 * divided into discrete simulation steps.
 *
 * The robot's motion model assumes a differential drive:
 * - If angular velocity is near zero, the robot moves straight.
 * - Otherwise, it follows a circular arc based on the radius derived from v/omega.
 *
 * @param v Forward velocity in centimeters per second (cm/s).
 * @param omega Angular velocity in radians per second (rad/s).
 * @param duration_sec Total simulation duration in seconds.
 * @param steps Number of discrete simulation steps.
 * @param theta_start_rad Initial orientation angle in radians.
 * @return std::vector<Pose2D> A vector of poses (x, y, theta) representing the trajectory.
 */
std::vector<ObstacleMapWidget::Pose2D> ObstacleMapWidget::computeGhostTrajectoryDiffDrive(
    double v,              // forward velocity (cm/s)
    double omega,          // angular velocity (rad/s)
    double duration_sec,   // total simulation time (s)
    int steps,             // number of simulation steps
    double theta_start_rad // initial orientation (rad)
) {
    std::vector<Pose2D> result;

    // Scale omega to adjust curvature effects (makes turns sharper if >1)
    omega *= m_curve_gain;

    double x = 0.0, y = 0.0, theta = theta_start_rad;
    double dt = duration_sec / steps;

    for (int i = 0; i <= steps; ++i) {
        // Save current pose
        result.push_back({x, y, theta});

        // Stop updating after last step
        if (i == steps)
            break;

        if (std::abs(omega) < 1e-6) {
            // Straight-line motion update
            x += v * std::cos(theta) * dt;
            y += v * std::sin(theta) * dt;
            // Orientation theta remains unchanged
        } else {
            // Circular arc motion update
            double R = v / omega;       // Turning radius
            double dtheta = omega * dt; // Angle increment

            // Update position based on circular arc formulas
            x += -R * std::sin(theta) + R * std::sin(theta + dtheta);
            y += R * std::cos(theta) - R * std::cos(theta + dtheta);
            theta += dtheta;
        }
    }

    return result;
}

/**
 * @brief Starts the ghost animation to visualize the predicted robot trajectory.
 *
 * This function computes the predicted future trajectory of the robot based on
 * the given speed and steering input, and then creates semi-transparent ghost
 * items along that trajectory in the scene. A timer is started to animate these
 * ghost items.
 *
 * @param speed_cm_s Forward velocity in centimeters per second.
 * @param steering_value Steering angular velocity in radians per second.
 */
void ObstacleMapWidget::startGhostAnimation(double speed_cm_s, double steering_value) {

    // Compute predicted ghost trajectory based on current parameters
    m_ghost_trajectory = computeGhostTrajectoryDiffDrive(speed_cm_s, steering_value,
                                                         m_ghost_duration, 30, m_robot_theta_rad);
    m_ghost_frame_index = 0;

    // Remove existing ghost items from the scene to avoid duplicates
    for (auto& item : m_ghostItems) {
        m_scene->removeItem(item);
        delete item;
    }
    m_ghostItems.clear();

    // Create new ghost items for each point in the trajectory
    for (int i = 0; i < static_cast<int>(m_ghost_trajectory.size()); ++i) {
        QGraphicsEllipseItem* ghost = new QGraphicsEllipseItem(0, 0, m_robot_size, m_robot_size);

        // Set the ghost color with transparency for visualization
        QColor transparent_color = m_ghost_color;
        transparent_color.setAlpha(30);
        ghost->setBrush(transparent_color);
        ghost->setPen(Qt::NoPen);

        ghost->setZValue(1); // Ensure ghosts are drawn beneath the actual robot
        ghost->setVisible(false);

        m_scene->addItem(ghost);
        m_ghostItems.push_back(ghost);
    }

    // Setup and start the timer to update ghost animation frames at ~100 Hz
    if (!m_ghost_timer) {
        m_ghost_timer = new QTimer(this);
        connect(m_ghost_timer, &QTimer::timeout, this,
                [this]() { updateGhostAnimation(m_robot_x_meters, m_robot_y_meters); });
    }
    m_ghost_timer->start(10);
}

/**
 * @brief Updates the ghost animation for the Ghost Mode.
 *
 * This function is called periodically (e.g., via a timer) and updates the position
 * and rotation of the current ghost item according to the precomputed trajectory.
 *
 * @param x_pos Current robot x position in world coordinates (meters).
 * @param y_pos Current robot y position in world coordinates (meters).
 */
void ObstacleMapWidget::updateGhostAnimation(double x_pos, double y_pos) {
    // Stop animation if all frames have been processed
    if (m_ghost_frame_index >= static_cast<int>(m_ghostItems.size())) {
        m_ghost_timer->stop();
        return;
    }

    // Get current pose from the trajectory
    const auto& pose = m_ghost_trajectory[m_ghost_frame_index];

    // Calculate position relative to the current robot location (meters to pixels)
    double pose_x_meters = pose.x / 100.0;
    double pose_y_meters = pose.y / 100.0;
    double base_x = x_pos + pose_x_meters;
    double base_y = y_pos + pose_y_meters;

    double theta = pose.theta;

    // Convert to Qt scene coordinates
    QPointF pos_px = worldToScene(base_x, base_y);

    // Update the ghost graphics item
    QGraphicsEllipseItem* ghost = m_ghostItems[m_ghost_frame_index];
    ghost->setRect(0, 0, m_robot_size, m_robot_size);
    ghost->setPos(pos_px.x() - m_robot_size / 2, pos_px.y() - m_robot_size / 2);
    ghost->setTransformOriginPoint(m_robot_size / 2, m_robot_size / 2);
    ghost->setRotation(-theta * 180.0 / M_PI);
    ghost->setVisible(true);

    // Advance to next frame
    m_ghost_frame_index++;
}

/**
 * @brief Deletes all ghost items from the scene and clears the ghost list.
 *
 * This function removes all ghost graphics items from the QGraphicsScene,
 * deletes them to free memory, and clears the container holding the pointers.
 */
void ObstacleMapWidget::deleteGhosts() {
    for (auto& item : m_ghostItems) {
        m_scene->removeItem(item);
        delete item;
    }
    m_ghostItems.clear();
}

/**
 * @brief Updates the speed trail by adding the current position and rendering a fading trail path.
 *
 * This function maintains a time-stamped history of recent positions and removes entries
 * older than the configured lifetime. It constructs a QPainterPath connecting all valid points
 * and updates a single QGraphicsPathItem in the scene to display the trail. The trail's color
 * and width are set using the pen.
 *
 * @param currentPosition The current position of the robot in scene coordinates.
 */

void ObstacleMapWidget::updateSpeedTrail(const QPointF& currentPosition) {
    QTime now = QTime::currentTime();

    // Add current position with timestamp to the history
    m_trailHistory.push_back(qMakePair(currentPosition, now));

    // Remove outdated trail points based on lifetime
    while (!m_trailHistory.empty() &&
           m_trailHistory.front().second.msecsTo(now) > m_trail_lifetime_ms) {
        m_trailHistory.pop_front();
    }

    // Create a path from the remaining trail points
    QPainterPath path;
    if (m_trailHistory.size() >= 2) {
        path.moveTo(m_trailHistory[0].first);

        for (unsigned long int i = 1; i < m_trailHistory.size(); ++i) {
            path.lineTo(m_trailHistory[i].first);
        }
    }

    // Configure pen (color and line style)
    QPen pen(m_trail_color);
    pen.setWidthF(3.0);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);

    // Update the trail graphics item
    m_trailItem->setPath(path);
    m_trailItem->setPen(pen);
}

/**
 * @brief Updates the widget's border color based on proximity to obstacles.
 *
 * This function sets the border color of the ObstacleMapWidget dynamically,
 * depending on the current minimum laser scan distance to an obstacle. The color
 * changes to indicate collision risk
 */
void ObstacleMapWidget::updateCollisionWarningBorder() {
    if (m_scan_available && m_collisionBorderMode) {
        QColor color;

        float distance = m_min_laser_distance;
        float smallestDistance = 0.15f;
        float longestDistance = 0.7f;

        if (distance < smallestDistance)
            distance = smallestDistance;
        else if (distance > longestDistance)
            distance = longestDistance;

        // Mapping distance 0.1 m to 0.7 m → 0.0 to 1.0
        float t = (distance - smallestDistance) /
                  (longestDistance - smallestDistance); // Normalized between 0 (close) and 1 (far)

        // Interpolate color:
        // From red (255,0,0) to yellow (255,255,0) to transparent (0,0,0,0)
        int red = 255;
        int green = static_cast<int>(255 * t); // Green increases with distance
        int blue = 0;
        int alpha = static_cast<int>(255 * (1.0f - t)); // Less visible when safe

        color = QColor(red, green, blue, alpha);

        QString style = QString("border: 20px solid rgba(%1,%2,%3,%4);")
                            .arg(color.red())
                            .arg(color.green())
                            .arg(color.blue())
                            .arg(color.alpha());

        this->setStyleSheet(style);
    } else {
        this->setStyleSheet("border: 1px solid lightgray;");
        return;
    }
}

/**
 * @brief Deletes all saved zones from the scene (for zoneMode)
 */
void ObstacleMapWidget::deleteZones() {
    // Remove current zone if available
    if (m_activeZone.graphicsItem) {
        m_scene->removeItem(m_activeZone.graphicsItem);
        delete m_activeZone.graphicsItem;
    }

    // Remove all saved zones
    for (auto it = m_savedZones.begin(); it != m_savedZones.end();) {
        if (it->valid && it->graphicsItem != nullptr) {
            m_scene->removeItem(it->graphicsItem);
            delete it->graphicsItem;
            it = m_savedZones.erase(it);
        } else {
            ++it;
        }
    }
}
