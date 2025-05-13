#ifndef TOUCH_SLIDER_VERTICAL_H
#define TOUCH_SLIDER_VERTICAL_H

#include <QWidget>
#include <QTouchEvent>
#include <QMouseEvent>
#include <QPainter>

class CustomTouchSliderVertical : public QWidget
{
    Q_OBJECT

public:
    explicit CustomTouchSliderVertical(QWidget *parent = nullptr);

    int getValue() const;  // Gibt den aktuellen Sliderwert zurück
    void setValue(int newValue);  // Setzt den Sliderwert und aktualisiert das Widget

protected:
    void paintEvent(QPaintEvent *) override;  // Zum Zeichnen des Sliders
    bool event(QEvent *event) override;  // Zum Verarbeiten von Touch- und Maus-Ereignissen
    void mousePressEvent(QMouseEvent *event) override;  // Verarbeitet Maus-Press-Ereignisse
    void mouseMoveEvent(QMouseEvent *event) override;  // Verarbeitet Maus-Bewegungs-Ereignisse
    void mouseReleaseEvent(QMouseEvent *) override;  // Optional für Release-Events

private:
    int m_value;  // Der aktuelle Wert des Sliders

    int mapToSliderValue(int y);  // Mapped die X-Position auf den Sliderwert
};

#endif // TOUCH_SLIDER_VERTICAL_H
