#include "../include/test_qt_pkg/obstacle_map_widget.h"
#include <QGraphicsRectItem>
#include <QRandomGenerator>
#include <QTimer>
#include <QVBoxLayout>  // wichtig

ObstacleMapWidget::ObstacleMapWidget(QWidget *parent) :
    QWidget(parent),
    scene_(new QGraphicsScene(this)),
    view_(new QGraphicsView(scene_, this))
{
    // Szenegröße bleibt fix, z. B. 800x600
    scene_->setSceneRect(0, 0, 800, 600);
    view_->setRenderHint(QPainter::Antialiasing);
    view_->setRenderHint(QPainter::SmoothPixmapTransform);

    // View in Layout einfügen
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(view_);
    layout->setContentsMargins(0, 0, 0, 0);
    setLayout(layout);

    // Roboter zeichnen
    robot_ = scene_->addEllipse(robot_x_ - 10, robot_y_ - 10, 20, 20, QPen(Qt::green), QBrush(Qt::green));
    
    // Linie mit Länge 10 als Ausrichtungspfeil (rot)  
    orientationLine_ = scene_->addLine(robot_x_, robot_y_, robot_x_ + 20, robot_y_, QPen(Qt::blue, 2));

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
    setupStaticObstacles();

    // Timer für Hindernisaktualisierungen
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &ObstacleMapWidget::updateObstacles);
    timer->start(20);
}


ObstacleMapWidget::~ObstacleMapWidget()
{
    delete scene_;
    delete view_;
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
    static int counter = 0;
    
    counter++;
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
