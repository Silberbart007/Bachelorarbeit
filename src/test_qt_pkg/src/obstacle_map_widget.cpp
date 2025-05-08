#include "../include/test_qt_pkg/obstacle_map_widget.h"
#include <QGraphicsRectItem>
#include <QRandomGenerator>
#include <QTimer>

ObstacleMapWidget::ObstacleMapWidget(QWidget *parent) :
    QWidget(parent),
    scene_(new QGraphicsScene(this)),
    view_(new QGraphicsView(scene_, this))
{
    // Setze die Szene
    scene_->setSceneRect(0, 0, 800, 600);
    view_->setRenderHint(QPainter::Antialiasing);
    view_->setRenderHint(QPainter::SmoothPixmapTransform);

    // Setze den Layout des Views
    view_->setGeometry(0, 0, 800, 600);

    // Timer für Hindernisaktualisierungen
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &ObstacleMapWidget::updateObstacles);
    timer->start(20);  // alle 1000 ms Hindernisse aktualisieren
}

ObstacleMapWidget::~ObstacleMapWidget()
{
    delete scene_;
    delete view_;
}

void ObstacleMapWidget::addObstacle(int x, int y, int width, int height)
{
    QGraphicsRectItem *rect = new QGraphicsRectItem(x, y, width, height);
    rect->setBrush(QBrush(Qt::red));  // Hindernis als rotes Rechteck
    scene_->addItem(rect);
}

void ObstacleMapWidget::updateObstacles()
{
    static int counter = 0;
    if (counter % 2 == 0) {
        addObstacle(QRandomGenerator::global()->bounded(800), QRandomGenerator::global()->bounded(600), 2, 2);
    }
    counter++;
}
