#ifndef OBSTACLEMAPWIDGET_H
#define OBSTACLEMAPWIDGET_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QTimer>

namespace Ui {
class ObstacleMapWidget;
}

class ObstacleMapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ObstacleMapWidget(QWidget *parent = nullptr);
    ~ObstacleMapWidget();

    void addObstacle(int x, int y, int width, int height);
    void updateObstacles();

protected:
    void resizeEvent(QResizeEvent *) override;

private:
    Ui::ObstacleMapWidget *ui;
    QGraphicsScene *scene_;
    QGraphicsView *view_;
};

#endif // OBSTACLEMAPWIDGET_H
