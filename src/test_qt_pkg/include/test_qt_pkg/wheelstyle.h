#ifndef WHEELSTYLE_H
#define WHEELSTYLE_H

#include <QColor>
#include <QFont>
#include <QString>

struct WheelStyle {
    QColor outerRingColor = Qt::darkGray;
    int outerRingWidth = 10;

    QColor spokeColor = Qt::black;
    int spokeWidth = 6;

    QColor centerCircleColor = Qt::gray;
    int centerCircleRadius = 50;

    QColor centerTextColor = Qt::black;
    QString centerText = "MV";
    QFont centerFont = QFont("Arial", 20, QFont::Bold);

    QVector<int> spokeAnglesDegrees = {90, 0, 180}; // z.B. unten, rechts, links

    qreal maxAngle = 450.0;
    bool isRound = true;
};

#endif // WHEELSTYLE_H
