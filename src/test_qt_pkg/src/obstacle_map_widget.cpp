#include "obstacle_map_widget.h" 

ObstacleMapWidget::ObstacleMapWidget(QWidget *parent) :
    QWidget(parent),
    m_scene(new QGraphicsScene(this)),
    m_view(new QGraphicsView(m_scene, this))
{
    m_drawing = false;
    m_following = false;
    m_temp_path_item = nullptr;
    m_temp_point_item = nullptr;
    m_robot_x_meters = 0.0; 
    m_robot_y_meters = 0.0; 
    m_robot_x_pixels = 0.0; 
    m_robot_y_pixels = 0.0; 
    m_robot_theta_rad = 0.0;
    m_beam_color = Qt::red;
    m_ghost_color = Qt::yellow;
    m_trail_color = Qt::cyan;
    m_laser_number = 270;

    m_view->setRenderHint(QPainter::Antialiasing);
    m_view->setRenderHint(QPainter::SmoothPixmapTransform);

    // Maus Events korrekt verarbeiten:
    m_view->viewport()->installEventFilter(this);

    // Gesten erkennen
    m_view->viewport()->grabGesture(Qt::PinchGesture);
    m_view->viewport()->grabGesture(Qt::PanGesture);

    //m_view->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    // View in Layout einfügen
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(m_view);
    layout->setContentsMargins(0, 0, 0, 0);
    setLayout(layout);

    // Default Szenenskalierung
    m_scene->setSceneRect(0, 0, 1000, 1000); // früh im Konstruktor

    // Timer für inertia initialisieren
    m_inertiaTimer.setInterval(30);
    connect(&m_inertiaTimer, &QTimer::timeout, this, &ObstacleMapWidget::handleInertia);

    // Parcour aufbauen (künstlich)
    //setupStaticObstacles();
}


ObstacleMapWidget::~ObstacleMapWidget()
{
    delete m_scene;
    delete m_view;
}

// Callback für Laser
void ObstacleMapWidget::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    m_current_scan = *msg;
    m_scan_available = true;
}

// Roboter Node setzen und Map/Roboter initiieren
void ObstacleMapWidget::setRobotNode(std::shared_ptr<RobotNode> robot_node) {
    m_robot_node = robot_node;

    // MapLoaded Funktion setzen
    if (m_robot_node->has_map()) {
        if (m_robot_node->has_map()) {
            QMetaObject::invokeMethod(this, [this]() {
                initializeRobot();
            }, Qt::QueuedConnection);
        }
    } else {
        m_robot_node->map_loaded = [this]() {
            QMetaObject::invokeMethod(this, [this]() {
                initializeRobot();
            }, Qt::QueuedConnection);
        };  
    }
}

// Nav2Client setzen
void ObstacleMapWidget::setNav2Node(std::shared_ptr<Nav2Client> nav2_node) {
    m_nav2_node = nav2_node;
}

// Roboter und Map startklar machen, wenn Map vorhanden ist
void ObstacleMapWidget::initializeRobot() {

    // Amcl Daten verarbeiten
    m_robot_node->on_amcl_pose_received = [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // Quaternion aus msg holen
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg->pose.pose.orientation, tf_q);

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

        m_robot_x_meters = msg->pose.pose.position.x;
        m_robot_y_meters = msg->pose.pose.position.y;
        //qDebug() << "x-real: " << m_robot_x_meters << " y-real " << m_robot_y_meters;
        m_robot_theta_rad = yaw;  // Winkel in Radiant
    };

    // Timer, der alle 50 ms die Roboterposition mit der letzten AMCL-Pose updatet
    QTimer* update_pose_timer = new QTimer(this);
    connect(update_pose_timer, &QTimer::timeout, this, [this]() {
        updateRobotPosition(m_robot_x_meters, m_robot_y_meters, m_robot_theta_rad);
    });
    update_pose_timer->start(10);

    // Roboter zeichnen
    m_robot_ellipse = m_scene->addEllipse(m_robot_x_pixels - m_robot_size / 2,
                           m_robot_y_pixels - m_robot_size / 2,
                           m_robot_size,
                           m_robot_size,
                           QPen(Qt::green),
                           QBrush(Qt::green));

    // Orientierungslinie, die nach vorne zeigt
    double end_x = m_robot_x_pixels + m_robot_size * cos(m_robot_theta_rad);
    double end_y = m_robot_y_pixels - m_robot_size * sin(m_robot_theta_rad); 

    m_orientation_line = m_scene->addLine(m_robot_x_pixels, m_robot_y_pixels, end_x, end_y, QPen(Qt::blue, 2));

    // Szene zentrieren auf die Map
    m_view->fitInView(m_scene->sceneRect(), Qt::KeepAspectRatio);

    updateObstacles();

    // Dieses Objekt an Nav2Client übergeben
    m_nav2_node->setObstacleMap(this);

    // Timer für Strahlenanzeige
    QTimer *beam_timer = new QTimer(this);
    connect(beam_timer, &QTimer::timeout, this, &ObstacleMapWidget::generateLaserBeams);
    beam_timer->start(100); // alle 100 ms

    // Timer für FollowMode
    QTimer* follow_timer = new QTimer(this);
    connect(follow_timer, &QTimer::timeout, this, &ObstacleMapWidget::followCurrentPoint);
    follow_timer->start(1000);  // alle 500 ms neues Ziel
}

// Funktion für Beam-Mode
void ObstacleMapWidget::generateLaserBeams() {
    if (m_scan_available && m_beamMode) {
        // Alte Strahlen löschen
        for (auto item : m_beam_items) {
            m_scene->removeItem(item);
            delete item;
        }
        m_beam_items.clear();

        int numRays = m_current_scan.ranges.size();
        int count = std::min(m_laser_number, numRays);  // Nicht mehr als vorhanden

        for (int i = 0; i < count; ++i) {
            int index = static_cast<int>((i / static_cast<float>(count - 1)) * (numRays - 1));
            float angle = m_current_scan.angle_min + index * m_current_scan.angle_increment;
            float dist = m_current_scan.ranges[index];
            if (!std::isfinite(dist)) continue;

            float theta = angle + m_robot_theta_rad;

            float endX = m_robot_x_pixels + dist * m_pixels_per_meter * std::cos(theta);
            float endY = m_robot_y_pixels - dist * m_pixels_per_meter * std::sin(theta);

            QGraphicsLineItem *line = m_scene->addLine(m_robot_x_pixels, m_robot_y_pixels, endX, endY, QPen(m_beam_color));
            line->setZValue(0);
            m_beam_items.push_back(line);
        }
    } else {
        for (auto item : m_beam_items) {
            m_scene->removeItem(item);
            delete item;
        }
        m_beam_items.clear();
    }
}

// Alle Zeichnungen (Also Punkte und Pfade) löschen
void ObstacleMapWidget::deleteAllDrawings() {
    // Gezeichnete Linie entfernen, falls vorhanden
    if (m_temp_path_item) {
        m_scene->removeItem(m_temp_path_item);
        delete m_temp_path_item;
        m_temp_path_item = nullptr;
    }
    // Einzelnen Punkt entfernen, falls vorhanden
    if (m_temp_point_item) {
        m_scene->removeItem(m_temp_point_item);
        delete m_temp_point_item;
        m_temp_point_item = nullptr;
    }
}

void ObstacleMapWidget::deleteGhosts() {
    // Items vorbereiten
    for (auto& item : m_ghostItems) {
        m_scene->removeItem(item);
        delete item;
    }
    m_ghostItems.clear();
}

// Inertia Bewegung
void ObstacleMapWidget::handleInertia() {
    const double max_speed = 0.4;
    const double friction = 0.99;

    // Stoppen, wenn Geschwindigkeit klein ist
    if ((std::abs(m_inertiaVelocity.x()) < 0.001 && std::abs(m_inertiaVelocity.y()) < 0.001) || !m_inertiaMode) {
        m_inertiaTimer.stop();
        // Optional: Bewegung vollständig stoppen
        m_robot_node->publish_velocity({0, 0}, m_robot_node->getRotationNormalized());
        return;
    }

    // Berechne normierte Geschwindigkeit direkt aus m_inertiaVelocity
    double vx_robot = std::clamp(m_inertiaVelocity.x(), -max_speed, max_speed);
    double vy_robot = std::clamp(m_inertiaVelocity.y(), -max_speed, max_speed);

    double norm_x = vx_robot / max_speed;
    double norm_y = vy_robot / max_speed;

    // Direkte Steuerung – kein Aufsummieren!
    geometry_msgs::msg::Vector3 velocity_msg;
    velocity_msg.x = std::clamp(norm_x, -1.0, 1.0);
    velocity_msg.y = std::clamp(norm_y, -1.0, 1.0);

    m_robot_node->publish_velocity({velocity_msg.x, velocity_msg.y}, m_robot_node->getRotationNormalized());

    // Geschwindigkeit dämpfen
    m_inertiaVelocity *= friction;
}

// Für Trail mode
void ObstacleMapWidget::updateSpeedTrail(const QPointF& currentPosition) {
    QTime now = QTime::currentTime();
    m_trailHistory.push_back(qMakePair(currentPosition, now));

    // Alte Punkte entfernen
    while (!m_trailHistory.empty() && m_trailHistory.front().second.msecsTo(now) > m_trail_lifetime_ms) {
        m_trailHistory.pop_front();
    }

    // Alte Linien entfernen
    for (auto line : m_trailLines) {
        m_scene->removeItem(line);
        delete line;
    }
    m_trailLines.clear();

    // Neue Linien erzeugen
    for (int i = 1; i < static_cast<int>(m_trailHistory.size()); ++i) {
        QPointF p1 = m_trailHistory[i - 1].first;
        QPointF p2 = m_trailHistory[i].first;
        int age = m_trailHistory[i].second.msecsTo(now);

        double opacity = 1.0 - static_cast<double>(age) / m_trail_lifetime_ms;
        QColor color = m_trail_color;
        color.setAlphaF(opacity);

        QPen pen(color);
        pen.setWidthF(3.0);

        QGraphicsLineItem* line = m_scene->addLine(QLineF(p1, p2), pen);
        m_trailLines.append(line);
    }
}



// Alle Events abfangen
bool ObstacleMapWidget::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == m_view->viewport()) {
        // Klick - anfang
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
            
            deleteAllDrawings();
            
            // Pfad Zeichnen - Modus
            if (m_drawPathMode) {
                m_drawing = true;
                m_path_points.clear();
                QPointF scenePos = m_view->mapToScene(mouseEvent->pos());
                m_path_points.push_back(scenePos);
            }
            // Follow Mode
            else if (m_followMode) {
                m_following = true;
                m_current_follow_point = m_view->mapToScene(mouseEvent->pos());
            }
            else if (m_inertiaMode) {
                m_inertiaTimer.stop();
                m_inertiaStart = m_view->mapToScene(mouseEvent->pos());
                m_inertiaStartTime = QTime::currentTime();
            }

            return true;  // Event verarbeitet
        }
        // Klick - Prozess
        else if (event->type() == QEvent::MouseMove) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);

            // Pfad zeichnen - Modus 
            if (m_drawPathMode) {
                if (!m_drawing) return false; // Nicht verarbeiten, wenn nicht am Zeichnen

                QPointF scenePos = m_view->mapToScene(mouseEvent->pos());
                m_path_points.push_back(scenePos);

                QPainterPath path;
                path.moveTo(m_path_points.first());
                for (const auto& pt : m_path_points)
                    path.lineTo(pt);

                if (m_temp_path_item) {
                    m_scene->removeItem(m_temp_path_item);
                    delete m_temp_path_item;
                }

                m_temp_path_item = m_scene->addPath(path, QPen(Qt::blue, 2));
            }
            // Follow Mode
            else if (m_followMode) {
                m_current_follow_point = m_view->mapToScene(mouseEvent->pos());
            }

            return true;  // Event verarbeitet
        }
        // Klick - Ende
        else if (event->type() == QEvent::MouseButtonRelease) {

            // Pfad zeichnen - Modus
            if (m_drawPathMode) {
                m_drawing = false;

                if (m_path_points.size() >= 2) {
                    pathDrawn(m_path_points);
                }
                // Nur ein Punkt (Follow Point mode)
                else if (m_path_points.size() == 1) {
                    // Ein einzelner Punkt
                    QPointF singlePoint = m_path_points.front();

                    // kleinen Kreis als Punkt zeichnen
                    qreal radius = 3.0;
                    m_temp_point_item = m_scene->addEllipse(
                        singlePoint.x() - radius,
                        singlePoint.y() - radius,
                        2 * radius,
                        2 * radius,
                        QPen(Qt::blue),
                        QBrush(Qt::blue)
                    );

                    pathDrawn(m_path_points);
                }

            return true;  // Event verarbeitet
            }
            // Follow Mode
            if (m_followMode) {
                m_following = false;
                
                // alle Goals canceln
                m_nav2_node->cancelGoalsPose();
                return true;
            }
            if (m_inertiaMode) {
                QPointF end = m_view->mapToScene(static_cast<QMouseEvent*>(event)->pos());
                int elapsed_ms = m_inertiaStartTime.msecsTo(QTime::currentTime());

                if (elapsed_ms > 0) {
                    double dt = elapsed_ms / 1000.0;
                    QPointF delta_pixels = QPointF(end.x() - m_inertiaStart.x(), m_inertiaStart.y() - end.y());
                    QPointF delta_meters = delta_pixels / m_pixels_per_meter;

                    QPointF velocity_mps = delta_meters / dt;

                    const double damping = 0.01;
                    velocity_mps *= damping;

                    m_inertiaVelocity = velocity_mps;
                    m_inertiaTimer.start();  // ruft regelmäßig handleInertia() auf
                }
                return true;
            }
        }
        // Touch Berührung
        else if (event->type() == QEvent::TouchBegin ||
                event->type() == QEvent::TouchUpdate ||
                event->type() == QEvent::TouchEnd) {

            QTouchEvent* touchEvent = static_cast<QTouchEvent*>(event);
            const auto& touchPoints = touchEvent->touchPoints();

            // Touch-Anfang
            if (event->type() == QEvent::TouchBegin) {
                
            }
            // Touch-Ende
            if (event->type() == QEvent::TouchEnd) {
    
            }
            // Stoppen bei mehreren Fingern im Inertia Mode
            if (m_inertiaMode) {
                if (touchPoints.count() >= 2) {
                    qDebug() << "Mehrere Finger erkannt – Bremsen";

                    // Roboter stoppen
                    geometry_msgs::msg::Twist stop;
                    stop.linear.x = 0;
                    stop.linear.y = 0;
                    stop.angular.z = 0;
                    m_robot_node->publish_velocity({stop.linear.x, stop.linear.y}, stop.angular.z);

                    // Optional: Trägheit beenden
                    m_inertiaVelocity = QPointF(0, 0);
                    if (m_inertiaTimer.isActive()) {
                        m_inertiaTimer.stop();
                    }

                    return true;
                }
            }
        }
        // Gesten
        else if (event->type() == QEvent::Gesture) {
            QGestureEvent* gestureEvent = static_cast<QGestureEvent*>(event);

            // Zoomen
            if (QGesture* g = gestureEvent->gesture(Qt::PinchGesture)) {
                QPinchGesture* pinch = static_cast<QPinchGesture*>(g);

                if (pinch->state() == Qt::GestureStarted || pinch->state() == Qt::GestureUpdated) {
                    qreal scaleFactor = pinch->scaleFactor();
                    m_view->scale(scaleFactor, scaleFactor);
                    return true;
                }
            }
            // Mit Finger verschieben (panning mit 3 Fingern)
            if (QGesture* g = gestureEvent->gesture(Qt::PanGesture)) {
                QPanGesture* pan = static_cast<QPanGesture*>(g);
                QPointF delta = pan->delta();

                // Optionales Debugging
                qDebug() << "Delta: " << delta;

                // aktuellen Mittelpunkt holen
                QPointF center = m_view->mapToScene(m_view->viewport()->rect().center());

                // neuen Mittelpunkt berechnen (delta invertieren für natürliches Wischen)
                QPointF newCenter = center - delta;

                // View auf neuen Mittelpunkt setzen
                m_view->centerOn(newCenter);

                return true;
            }
        }
    }
    // Alle anderen Events normal weiterreichen
    return QWidget::eventFilter(obj, event);
}

void ObstacleMapWidget::updateRobotPosition(double x, double y, double theta)
{
    if (!m_robot_node->has_map()) return;

    //qDebug() << "X vor worldtoscene: " << x;
    QPointF robo_pos = worldToScene(x, y);
    m_robot_x_pixels = robo_pos.x();
    m_robot_y_pixels = robo_pos.y();
    m_robot_theta_rad = theta;
    //qDebug() << "x nach Worldtoscene: " << m_robot_x_pixels;

    if (m_robot_ellipse) {
        // Roboter vorne anzeigen
        m_robot_ellipse->setZValue(2);

        // Größe und Form des Roboters (immer bei 0,0 relativ zum Item)
        m_robot_ellipse->setRect(0, 0, m_robot_size, m_robot_size);

        // Position des Roboters in der Szene (Mittelpunkt ausrichten)
        m_robot_ellipse->setPos(m_robot_x_pixels - m_robot_size / 2, m_robot_y_pixels - m_robot_size / 2);

        m_robot_ellipse->setTransformOriginPoint(m_robot_size / 2, m_robot_size / 2);
        m_robot_ellipse->setRotation(-m_robot_theta_rad * 180.0 / M_PI);
        //qDebug() << "robot_theta (rad):" << m_robot_theta_rad << "rot (deg):" << (-m_robot_theta_rad * 180.0 / M_PI);

    }
    if (m_orientation_line) {
        double length = 20;
        double endX = m_robot_x_pixels + length * std::cos(m_robot_theta_rad);
        double endY = m_robot_y_pixels - length * std::sin(m_robot_theta_rad); 
        m_orientation_line->setLine(m_robot_x_pixels, m_robot_y_pixels, endX, endY);

        // Orientierung ganz Vorne anzeigen
        m_orientation_line->setZValue(3);
    }

    // Ghost Mode
    if (m_ghostMode) {
        double current_speed = m_robot_node->getSpeed().x * 100.0;
        double current_rot = m_robot_node->getRotation();
        double current_steering = current_rot;

        if (std::abs(m_last_speed - current_speed) > 1e-2 || std::abs(m_last_steering - current_steering) > 1e-2) {
            startGhostAnimation(current_speed, -current_steering);
            m_last_speed = current_speed;
            m_last_steering = current_steering;
        }
    } else {
        deleteGhosts();
    }

    // Trail Mode
    if (m_trailMode) {
        QPointF scenePos = worldToScene(x,y);
        updateSpeedTrail(scenePos);
    }
    
}


void ObstacleMapWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    m_view->fitInView(m_scene->sceneRect(), Qt::KeepAspectRatio); 
    m_view->centerOn(m_robot_x_pixels, m_robot_y_pixels);
}

// Map Koordinaten in Qt-Pixel koordinaten ausrechnen
QPointF ObstacleMapWidget::worldToScene(double x_m, double y_m) {
    double scene_x = x_m * m_pixels_per_meter; 
    double scene_y = -y_m * m_pixels_per_meter;
    
    
    // qDebug() << "worldToScene: x_m=" << x_m << " y_m=" << y_m
    //          << " -> scene_x=" << scene_x << " scene_y=" << scene_y;

    return QPointF(scene_x, scene_y);
}


QPointF ObstacleMapWidget::sceneToMapCoordinates(const QPointF& scene_pos) {
    double x_m = scene_pos.x() / m_pixels_per_meter;
    double y_m = -scene_pos.y() / m_pixels_per_meter;

    return QPointF(x_m, y_m);
}


void ObstacleMapWidget::updateObstaclesFromMap()
{
    if (!m_robot_node->has_map()) return;

    // Szene leeren
    QList<QGraphicsItem*> items = m_scene->items();
    for (auto item : items) {
        if (QGraphicsRectItem* rect = dynamic_cast<QGraphicsRectItem*>(item)) {
            m_scene->removeItem(rect);
            delete rect;
        }
    }

    // Map laden
    m_map = m_robot_node->get_map();
    int width = m_map.info.width;
    int height = m_map.info.height;
    double resolution = m_map.info.resolution;

    // Skalierung in Pixeln
    double cellSize = resolution * m_pixels_per_meter;

    // Ursprung der Karte in Weltkoordinaten (Meter)
    double origin_x = m_map.info.origin.position.x;
    double origin_y = m_map.info.origin.position.y;

    // Szenegröße in Pixeln
    double scene_width = width * cellSize;
    double scene_height = height * cellSize;

    // Szene-Rechteck korrekt setzen (y invertiert, damit Qt nach unten wächst)
    double scene_left = origin_x * m_pixels_per_meter;
    double scene_top = -origin_y * m_pixels_per_meter - scene_height;

    m_scene->setSceneRect(scene_left, scene_top, scene_width, scene_height);

    // Hindernisse einzeichnen
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int i = x + y * width;
            if (m_map.data[i] == 100) {
                // Koordinaten in Szene umrechnen
                double map_x = origin_x + x * resolution;
                double map_y = origin_y + y * resolution;
                
                QPointF scene_pos = worldToScene(map_x, map_y);

                // Zelle zeichnen
                addObstacle(scene_pos.x(), scene_pos.y(), cellSize, cellSize);
            }
        }
    }

    m_view->fitInView(m_scene->sceneRect(), Qt::KeepAspectRatio);
    m_view->centerOn(m_robot_x_pixels, m_robot_y_pixels);
}



void ObstacleMapWidget::addObstacle(double x, double y, double width, double height)
{
    QGraphicsRectItem *rect = new QGraphicsRectItem(x, y, width, height);
    rect->setBrush(QBrush(Qt::red));  // Hindernis als rotes Rechteck
    m_scene->addItem(rect);
}

// Künstlich Parcour aufbauen
void ObstacleMapWidget::setupStaticObstacles()
{
    // Beispiel Parcours: Wände und Hindernisse (Positionen & Größen anpassen)
    addObstacle(100, 100, 600, 20);  // obere Wand
    addObstacle(100, 480, 600, 20);  // untere Wand
    addObstacle(100, 100, 20, 400);  // linke Wand
    addObstacle(680, 100, 20, 400);  // rechte Wand

    // Ein paar Hindernisse im Innenbereich
    addObstacle(300, 200, 50, 150);
    addObstacle(450, 350, 100, 30);
    addObstacle(550, 150, 20, 150);
}

void ObstacleMapWidget::updateObstacles()
{
    updateObstaclesFromMap();
}

bool ObstacleMapWidget::isNearObstacle(float x, float y)
{
    // Roboter als Kreis mit Mittelpunkt (x,y) und Radius, z.B. 10
    const float robotRadius = 10.0f;

    for (auto item : m_scene->items()) {
        auto rectItem = dynamic_cast<QGraphicsRectItem*>(item);
        if (!rectItem) continue;

        QRectF rect = rectItem->rect();

        // Abstand Kreis-Rechteck minimal berechnen
        float closestX = std::max(float(rect.left()), std::min(x, float(rect.right())));
        float closestY = std::max(float(rect.top()), std::min(y, float(rect.bottom())));

        float dx = x - closestX;
        float dy = y - closestY;

        float distance = std::sqrt(dx*dx + dy*dy);

        if (distance < (robotRadius + EMERGENCY_STOP_RADIUS)) {
            return true;  // Not-Aus nötig
        }
    }

    return false;
}   

// Im Follow Mode aktuelle Finger/Maus-Position verfolgen (wird jede ...ms geschickt)
void ObstacleMapWidget::followCurrentPoint() {
    // Roboter fährt bei keinem Berührungspunkt "zu sich selbst"
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

    // Punkt an Client schicken (ausführen)
    m_nav2_node->sendGoal(goal);
}

std::vector<ObstacleMapWidget::Pose2D> ObstacleMapWidget::computeGhostTrajectoryDiffDrive(
    double v,                       // Vorwärtsgeschwindigkeit in cm/s
    double omega,                   // Drehgeschwindigkeit in rad/s
    double duration_sec,            // Simulationsdauer
    int steps,                      // Simulationsschritte
    double theta_start_rad          // Anfangsorientierung
) {
    std::vector<Pose2D> result;

    // omega skalieren für weitere Effekte (schärfere Kurve)
    omega *= m_curve_gain;

    double x = 0.0, y = 0.0, theta = theta_start_rad;
    double dt = duration_sec / steps;

    for (int i = 0; i <= steps; ++i) {
        result.push_back({x, y, theta});

        // Nur updaten, wenn wir noch Schritte vor uns haben
        if (i == steps) break;

        if (std::abs(omega) < 1e-6) {
            // Geradeausfahrt
            x += v * std::cos(theta) * dt;
            y += v * std::sin(theta) * dt;
            // theta bleibt gleich
        } else {
            // Kreisbewegung
            double R = v / omega;
            double dtheta = omega * dt;
            x += -R * std::sin(theta) + R * std::sin(theta + dtheta);
            y +=  R * std::cos(theta) - R * std::cos(theta + dtheta);
            theta += dtheta;
        }
    }

    return result;
}


// Für GhostMode: Animation starten
void ObstacleMapWidget::startGhostAnimation(double speed_cm_s, double steering_value) {

    m_ghost_trajectory = computeGhostTrajectoryDiffDrive(speed_cm_s, steering_value, m_ghost_duration, 60, m_robot_theta_rad);
    m_ghost_frame_index = 0;

    // Items vorbereiten
    for (auto& item : m_ghostItems) {
        m_scene->removeItem(item);
        delete item;
    }
    m_ghostItems.clear();

    for (int i = 0; i < static_cast<int>(m_ghost_trajectory.size()); ++i) {
        QGraphicsEllipseItem* ghost = new QGraphicsEllipseItem(0, 0, m_robot_size, m_robot_size);
        QColor transparent_color = m_ghost_color;
        m_ghost_color.setAlpha(100);
        ghost->setBrush(transparent_color); // halbtransparent
        ghost->setPen(Qt::NoPen);
        ghost->setZValue(1); // unter echtem Roboter
        ghost->setVisible(false);
        m_scene->addItem(ghost);
        m_ghostItems.push_back(ghost);
    }

    if (!m_ghost_timer) {
        m_ghost_timer = new QTimer(this);
        connect(m_ghost_timer, &QTimer::timeout, this, [this]() {
                    updateGhostAnimation(m_robot_x_meters, m_robot_y_meters);
                });
    }
    m_ghost_timer->start(10);
}

// Für GhostMode: Animieren
void ObstacleMapWidget::updateGhostAnimation(double x_pos, double y_pos) {
    if (m_ghost_frame_index >= static_cast<int>(m_ghostItems.size())) {
        m_ghost_timer->stop();
        return;
    }

    const auto& pose = m_ghost_trajectory[m_ghost_frame_index];

    double base_x = x_pos + pose.x / m_pixels_per_meter;
    double base_y = y_pos + pose.y / m_pixels_per_meter;
    double theta = m_robot_theta_rad + pose.theta;

    QPointF pos_px = worldToScene(base_x, base_y);

    QGraphicsEllipseItem* ghost = m_ghostItems[m_ghost_frame_index];
    ghost->setRect(0, 0, m_robot_size, m_robot_size);
    ghost->setPos(pos_px.x() - m_robot_size / 2, pos_px.y() - m_robot_size / 2);
    ghost->setTransformOriginPoint(m_robot_size / 2, m_robot_size / 2);
    ghost->setRotation(-theta * 180.0 / M_PI);
    ghost->setVisible(true);

    m_ghost_frame_index++;
}

// Pfad zu Ende gezeichnet
void ObstacleMapWidget::pathDrawn(const QVector<QPointF>& points) {
    m_current_path = resamplePath(points, 5.0);
    m_current_target_index = 0;
    goToNextPoint();
}

// Pfad zeichnen - Punkt zeichnen ausführen (je nach größe des Pfads)
void ObstacleMapWidget::goToNextPoint() {
    if (m_current_target_index >= m_current_path.size()) {
        m_robot_node->publish_velocity({0.0, 0.0}, 0.0);
        return;
    }

    // Pose-Client (Nur 1 Punkt) 
    if (m_current_path.size() == 1) {
        QPointF target = m_current_path[m_current_target_index];
        QPointF target_world = sceneToMapCoordinates(target);

        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = m_nav2_node->now();

        goal.pose.position.x = target_world.x();
        goal.pose.position.y = target_world.y();
        goal.pose.position.z = 0.0;
        //goal.pose.orientation.w = 1.0; // keine Drehung

        // Punkt an Client schicken (ausführen)
        m_nav2_node->sendGoal(goal);
    }
    // FollowPath-Client 
    else {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = m_nav2_node->now();

        for (const QPointF &pt : m_current_path) {
            QPointF world = sceneToMapCoordinates(pt);

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = world.x();
            pose.pose.position.y = world.y();
            pose.pose.position.z = 0.0;
            //pose.pose.orientation.w = 1.0;  // keine Drehung
            path.poses.push_back(pose);
        }

        // Pfad an Client schicken (ausführen)
        m_nav2_node->sendPath(path);
    }
}

QVector<QPointF> ObstacleMapWidget::resamplePath(const QVector<QPointF>& originalPoints, double spacing = 5.0) {
    if (originalPoints.size() < 2)
        return originalPoints;

    QVector<QPointF> newPoints;
    newPoints.push_back(originalPoints.first());

    double accumulatedDist = 0.0;
    for (int i = 1; i < originalPoints.size(); ++i) {
        QPointF p0 = originalPoints[i - 1];
        QPointF p1 = originalPoints[i];
        double segmentLength = QLineF(p0, p1).length();

        while (accumulatedDist + segmentLength >= spacing) {
            double t = (spacing - accumulatedDist) / segmentLength;
            QPointF newPoint = p0 + t * (p1 - p0);
            newPoints.push_back(newPoint);

            // neuer Startpunkt für nächsten Interpolationsschritt
            p0 = newPoint;
            segmentLength = QLineF(p0, p1).length();
            accumulatedDist = 0;
        }
        accumulatedDist += segmentLength;
    }
    // letzten Punkt anfügen
    if (!newPoints.last().isNull() && newPoints.last() != originalPoints.last())
        newPoints.push_back(originalPoints.last());

    return newPoints;
}




