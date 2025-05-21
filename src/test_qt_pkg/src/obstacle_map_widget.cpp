#include "obstacle_map_widget.h" 

ObstacleMapWidget::ObstacleMapWidget(QWidget *parent) :
    QWidget(parent),
    scene_(new QGraphicsScene(this)),
    view_(new QGraphicsView(scene_, this))
{
    drawing_ = false;
    temp_path_item_ = nullptr;
    temp_point_item_ = nullptr;

    // Szenegröße bleibt fix, z. B. 800x600
    scene_->setSceneRect(0, 0, 800, 600);
    view_->setRenderHint(QPainter::Antialiasing);
    view_->setRenderHint(QPainter::SmoothPixmapTransform);

    // Maus Events korrekt verarbeiten:
    view_->viewport()->installEventFilter(this);

    // Gesten erkennen
    view_->viewport()->grabGesture(Qt::PinchGesture);
    //view_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    // View in Layout einfügen
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(view_);
    layout->setContentsMargins(0, 0, 0, 0);
    setLayout(layout);

    // Roboter zeichnen
    robot_ = scene_->addEllipse(robot_x_ - 10, robot_y_ - 10, 20, 20, QPen(Qt::green), QBrush(Qt::green));
    
    // Linie mit Länge 10 als Ausrichtungspfeil (rot)  
    orientationLine_ = scene_->addLine(robot_x_, robot_y_, robot_x_ + 20, robot_y_, QPen(Qt::blue, 2));

    // Roboterbewegung initiieren
    QTimer *move_timer = new QTimer(this);
    connect(move_timer, &QTimer::timeout, this, [this]() {
        double dt = 0.05; // 50 ms

        // Geschwindigkeit und Drehgeschwindigkeit vom RobotNode holen
        RobotNode::RobotSpeed v = m_robot_node->getSpeed(); // lokale Geschwindigkeit (vorwärts x, seitlich y)
        double omega = m_robot_node->getRotation(); // Winkelgeschwindigkeit in rad/s

        // Orientierung nur aktualisieren, wenn Drehgeschwindigkeit ungleich 0 ist
        if (std::abs(omega) > 1e-6) {
            double direction = (v.x >= 0) ? 1.0 : -1.0;
            robot_theta_ += direction * omega * dt;
            if (robot_theta_ > 2 * M_PI) robot_theta_ -= 2 * M_PI;
            else if (robot_theta_ < 0) robot_theta_ += 2 * M_PI;
        }

        // Lokale Geschwindigkeit in Weltkoordinaten umrechnen
        double dx = v.x * std::cos(robot_theta_) - v.y * std::sin(robot_theta_);
        double dy = v.x * std::sin(robot_theta_) + v.y * std::cos(robot_theta_);

        // Neue Position berechnen
        robot_x_ += dx * dt;
        robot_y_ += dy * dt;

        updateRobotPosition(robot_x_, robot_y_, robot_theta_);
    });
    move_timer->start(50);

    // Parcour aufbauen (künstlich)
    //setupStaticObstacles();

    // Timer für Hindernisaktualisierungen
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &ObstacleMapWidget::updateObstacles);
    timer->start(100);

    // Timer für Strahlenanzeige
    QTimer *beam_timer = new QTimer(this);
    connect(beam_timer, &QTimer::timeout, this, &ObstacleMapWidget::generateDummyData);
    beam_timer->start(100); // alle 100 ms
}


ObstacleMapWidget::~ObstacleMapWidget()
{
    delete scene_;
    delete view_;
}


void ObstacleMapWidget::generateDummyData() {
    if (beamMode_) {
        latestDistances_.clear();
        for (int i = 0; i < 10; ++i) {
            latestDistances_.push_back(0.5f + static_cast<float>(rand()) / RAND_MAX * 1.5f);
        }

        // Alte Strahlen löschen
        for (auto item : beam_items_) {
            scene_->removeItem(item);
            delete item;
        }
        beam_items_.clear();

        // Neue Strahlen hinzufügen
        int numRays = 10;
        float maxLength = 30.0f;
        float angleStep = M_PI / (numRays - 1);

        for (int i = 0; i < numRays; ++i) {
            float angle = -M_PI/2 + i * angleStep;
            float dist = latestDistances_[i];
            float length = dist * maxLength;

            float endX = robot_x_ + length * std::cos(angle + robot_theta_);
            float endY = robot_y_ + length * std::sin(angle + robot_theta_);

            QGraphicsLineItem *line = scene_->addLine(robot_x_, robot_y_, endX, endY, QPen(Qt::red));
            beam_items_.push_back(line);
        }
    } else {
        for (auto item : beam_items_) {
            scene_->removeItem(item);
            delete item;
        }
        beam_items_.clear();
    }
}

// Alle Events abfangen -> Pfad zeichnen
bool ObstacleMapWidget::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == view_->viewport()) {
        // Klick - anfang
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);

            // Pfad Zeichnen - Modus
            if (drawPathMode_) {
                drawing_ = true;
                path_points_.clear();
                QPointF scenePos = view_->mapToScene(mouseEvent->pos());
                path_points_.push_back(scenePos);
                if (temp_path_item_) {
                    scene_->removeItem(temp_path_item_);
                    delete temp_path_item_;
                    temp_path_item_ = nullptr;
                }
            }

            return true;  // Event verarbeitet
        }
        // Klick - Prozess
        else if (event->type() == QEvent::MouseMove) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);

            // Pfad zeichnen - Modus 
            if (drawPathMode_) {
                if (!drawing_) return false; // Nicht verarbeiten, wenn nicht am Zeichnen

                QPointF scenePos = view_->mapToScene(mouseEvent->pos());
                path_points_.push_back(scenePos);

                QPainterPath path;
                path.moveTo(path_points_.first());
                for (const auto& pt : path_points_)
                    path.lineTo(pt);

                if (temp_path_item_) {
                    scene_->removeItem(temp_path_item_);
                    delete temp_path_item_;
                }

                temp_path_item_ = scene_->addPath(path, QPen(Qt::blue, 2));
            }

            return true;  // Event verarbeitet
        }
        // Klick - Ende
        else if (event->type() == QEvent::MouseButtonRelease) {

            // Pfad zeichnen - Modus
            if (drawPathMode_) {
                drawing_ = false;

                // Einzelnen Punkt entfernen, falls vorhanden
                if (temp_point_item_) {
                    scene_->removeItem(temp_point_item_);
                    delete temp_point_item_;
                    temp_point_item_ = nullptr;
                }

                // Gezeichnete Linie entfernen, falls vorhanden
                if (temp_path_item_) {
                    scene_->removeItem(temp_path_item_);
                    delete temp_path_item_;
                    temp_path_item_ = nullptr;
                }

                if (path_points_.size() >= 2) {
                    pathDrawn(path_points_);
                }
                // Nur ein Punkt (Follow Point mode)
                else if (path_points_.size() == 1) {
                    // Ein einzelner Punkt
                    QPointF singlePoint = path_points_.front();

                    // kleinen Kreis als Punkt zeichnen
                    qreal radius = 3.0;
                    temp_point_item_ = scene_->addEllipse(
                        singlePoint.x() - radius,
                        singlePoint.y() - radius,
                        2 * radius,
                        2 * radius,
                        QPen(Qt::blue),
                        QBrush(Qt::blue)
                    );

                    pathDrawn(path_points_);
                }

            return true;  // Event verarbeitet
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
                    
                    // -> Zoom auf die View anwenden
                    view_->scale(scaleFactor, scaleFactor);

                    return true;  // Event verarbeitet
                }
            }
        }
    }
    // Alle anderen Events normal weiterreichen
    return QWidget::eventFilter(obj, event);
}

void ObstacleMapWidget::updateRobotPosition(double x, double y, double theta)
{
    // Begrenzung wie gehabt
    if (x < 10) x = 10;
    if (x > 790) x = 790;
    if (y < 10) y = 10;
    if (y > 590) y = 590;

    if (isNearObstacle(x, y)) {
        qDebug() << "Emergency Stop! Roboter zu nah am Hindernis.";
        robot_x_ = 400.0;
        robot_y_ = 300.0;
    } else {
        robot_x_ = x;
        robot_y_ = y;
        robot_theta_ = theta;

        if (robot_) {
            robot_->setRect(robot_x_ - 10, robot_y_ - 10, 20, 20);
        }
        if (orientationLine_) {
            // Linie vom Mittelpunkt zu Ausrichtungspunkt neu setzen
            double length = 20;
            double endX = robot_x_ + length * std::cos(robot_theta_);
            double endY = robot_y_ + length * std::sin(robot_theta_);
            orientationLine_->setLine(robot_x_, robot_y_, endX, endY);
        }
    }
}


void ObstacleMapWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    view_->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);  // optional: oder Qt::IgnoreAspectRatio
}

QImage ObstacleMapWidget::toImage()
{
    m_map = m_robot_node->get_map();

    if (!m_robot_node->has_map()) return QImage();

    int width = m_map.info.width;
    int height = m_map.info.height;
    QImage image(width, height, QImage::Format_RGB888);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int i = x + (height - y - 1) * width;  // Y invertieren, damit Ursprung unten links

            int val = m_map.data[i];
            QColor color;

            if (val == 0)         color = Qt::white;     // frei
            else if (val == 100)  color = Qt::black;     // belegt
            else                  color = Qt::gray;      // unbekannt

            image.setPixelColor(x, y, color);
        }
    }

    return image;
}


void ObstacleMapWidget::updateObstaclesFromMap()
{
    if (!m_robot_node->has_map()) return;

    // Erst alle bisherigen Hindernisse löschen
    QList<QGraphicsItem*> items = scene_->items();
    for (auto item : items) {
        // Optional: nur rote Rechtecke löschen, falls du statische Wände behalten willst
        if (QGraphicsRectItem* rect = dynamic_cast<QGraphicsRectItem*>(item)) {
            scene_->removeItem(rect);
            delete rect;
        }
    }

    // Map auslesen
    m_map = m_robot_node->get_map();
    int width = m_map.info.width;
    int height = m_map.info.height;

    // Zellgröße in Szene (Pixel) bestimmen (anpassen!)
    double cellSize = 5.0;  // Beispiel: 1 Zelle = 5x5 Pixel

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int i = x + y * width;  // hier normale Reihenfolge

            if (m_map.data[i] == 100) { // Hindernis
                // Rechteck an Position zeichnen
                addObstacle(x * cellSize, y * cellSize, cellSize, cellSize);
            }
        }
    }
}

void ObstacleMapWidget::addObstacle(int x, int y, int width, int height)
{
    QGraphicsRectItem *rect = new QGraphicsRectItem(x, y, width, height);
    rect->setBrush(QBrush(Qt::red));  // Hindernis als rotes Rechteck
    scene_->addItem(rect);
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

    for (auto item : scene_->items()) {
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

// Pfad zu Ende gezeichnet - Signal
void ObstacleMapWidget::pathDrawn(const QVector<QPointF>& points) {
    current_path_ = resamplePath(points, 7.0);  // alle 7 Pixel ein Punkt
    current_target_index_ = 0;
    goToNextPoint();
}

void ObstacleMapWidget::goToNextPoint() {
    if (current_target_index_ >= current_path_.size()) {
        m_robot_node->publish_velocity({0.0, 0.0}, 0.0);
        return;
    }

    QPointF target = current_path_[current_target_index_];
    double dx = target.x() - robot_x_;
    double dy = target.y() - robot_y_;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < 5.0) {  // Zielpunkt erreicht
        current_target_index_++;
        if (current_target_index_ >= current_path_.size()) {
            m_robot_node->publish_velocity({0.0, 0.0}, 0.0);
            return;
        }
        target = current_path_[current_target_index_];
        dx = target.x() - robot_x_;
        dy = target.y() - robot_y_;
    }

    double angle_to_target = std::atan2(dy, dx);
    double angle_diff = angle_to_target - robot_theta_;
    angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff)); // Normalisieren

    double gain_rot = 2.0;
    double rotation = std::tanh(gain_rot * angle_diff);  // sanfte Begrenzung auf [-1, 1]

    // Geschwindigkeit
    double max_speed = 1.0;
    double gain = 2.0;  // Steuerparameter
    double angle_factor = 1.0 - std::tanh(gain * std::abs(angle_diff));
    double speed = max_speed * std::max(angle_factor, 0.1);  // Niemals ganz 0

    RobotNode::RobotSpeed cmd;
    cmd.x = speed;
    cmd.y = 0.0;

    m_robot_node->publish_velocity(cmd, rotation);

    // Timer für nächsten Aufruf
    QTimer::singleShot(50, this, &ObstacleMapWidget::goToNextPoint);
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
    // Optional: letzten Punkt anfügen
    if (!newPoints.last().isNull() && newPoints.last() != originalPoints.last())
        newPoints.push_back(originalPoints.last());

    return newPoints;
}




