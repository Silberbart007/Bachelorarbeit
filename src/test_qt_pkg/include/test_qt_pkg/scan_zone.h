#ifndef SCANZONE_H
#define SCANZONE_H

#include <QPointF>
#include <QPolygonF>
#include <QtMath>

/**
 * @brief Represents a scanning zone defined by a regular polygon.
 * 
 * A ScanZone is created by the user through a multi-touch gesture.
 * The number of corners is determined by the number of fingers used,
 * and the size is defined by the distance from the center to the gesture edge.
 */
struct ScanZone {
    /**
     * @brief The center of the zone in widget coordinates.
     */
    QPointF center;

    /**
     * @brief The radius of the zone (distance from center to vertices).
     */
    qreal radius;

    /**
     * @brief Number of corners of the polygon (e.g. 3 = triangle, 4 = square).
     */
    int corners;

    /**
     * @brief Whether the zone is currently valid (drawn and confirmed).
     */
    bool valid = false;

    /// @brief Corresponding polygon item
    QGraphicsPolygonItem* graphicsItem = nullptr;

    /**
     * @brief Returns the polygon shape of this zone.
     * 
     * The method generates a regular polygon with the given number
     * of corners, centered at `center`, with radius `radius`.
     * 
     * @return QPolygonF The generated polygon.
     */
    QPolygonF polygon() const {
        QPolygonF poly;
        for (int i = 0; i < corners; ++i) {
            double angle = (2.0 * M_PI * i) / corners;
            QPointF pt(center.x() + radius * std::cos(angle),
                       center.y() + radius * std::sin(angle));
            poly << pt;
        }
        return poly;
    }
};

#endif // SCANZONE_H
