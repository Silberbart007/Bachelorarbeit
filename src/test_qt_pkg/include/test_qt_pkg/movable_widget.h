#ifndef MOVABLE_WIDGET_H
#define MOVABLE_WIDGET_H

#include <QWidget>


class MovableWidget : public QWidget {
    Q_OBJECT

public:
    explicit MovableWidget(QWidget *parent = nullptr);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

private:
    QPoint lastMousePosition;
};


#endif
