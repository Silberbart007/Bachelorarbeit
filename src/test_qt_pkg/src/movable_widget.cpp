#include "../include/test_qt_pkg/movable_widget.h"
#include <QMouseEvent>

MovableWidget::MovableWidget(QWidget *parent)
    : QWidget(parent) {
    setMouseTracking(true);
}

void MovableWidget::mousePressEvent(QMouseEvent *event) {
    lastMousePosition = event->globalPos() - this->frameGeometry().topLeft();
    event->accept();
}

void MovableWidget::mouseMoveEvent(QMouseEvent *event) {
    if (event->buttons() & Qt::LeftButton) {
        move(event->globalPos() - lastMousePosition);
        event->accept();
    }
}
