#include "touch_test.h"
#include <QTouchEvent>
#include <QPainter>
#include <QDebug>

TouchTestWidget::TouchTestWidget(QWidget *parent)
    : QWidget(parent), touchLabel(new QLabel(this))
{
    setWindowTitle("Multi-Touch Test");
    setGeometry(100, 100, 800, 600);  // Setze die Größe des Fensters
    touchLabel->setGeometry(10, 10, 200, 30);  // Setze die Position und Größe des Labels
    touchLabel->setText("Touch the screen to test multi-touch");
    touchLabel->setStyleSheet("QLabel { color : black; font: bold 14px; }");
}

bool TouchTestWidget::event(QEvent *event)
{
    if (event->type() == QEvent::TouchBegin || event->type() == QEvent::TouchUpdate || event->type() == QEvent::TouchEnd) {
        QTouchEvent *touchEvent = static_cast<QTouchEvent *>(event);
        QList<QTouchEvent::TouchPoint> touchPoints = touchEvent->touchPoints();

        // Zeichne die Positionen der Finger auf dem Bildschirm
        QString touchInfo = "Touch Points: ";
        for (const QTouchEvent::TouchPoint &point : touchPoints) {
            touchInfo += QString("ID: %1, x: %2, y: %3; ")
                             .arg(point.id())
                             .arg(point.pos().x())
                             .arg(point.pos().y());
        }

        // Aktualisiere das Label, um die Positionen der Touch-Punkte anzuzeigen
        touchLabel->setText(touchInfo);

        // Optionale Ausgabe im Debugging-Log
        qDebug() << touchInfo;

        return true;  // Touch-Ereignis wurde verarbeitet
    }

    return QWidget::event(event);  // Standardverhalten für andere Ereignisse
}
