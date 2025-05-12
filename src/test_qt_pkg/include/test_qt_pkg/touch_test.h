#ifndef TOUCHTESTWIDGET_H
#define TOUCHTESTWIDGET_H

#include <QWidget>
#include <QTouchEvent>
#include <QLabel>

class TouchTestWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TouchTestWidget(QWidget *parent = nullptr);

protected:
    bool event(QEvent *event) override;

private:
    QLabel *touchLabel;
};

#endif // TOUCHTESTWIDGET_H
