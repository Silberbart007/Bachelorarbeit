#pragma once
#include "stop_button.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWidget>
#include <QDebug>

class ControlWidgetWrapper : public QWidget {
    Q_OBJECT

  public:
    explicit ControlWidgetWrapper(QWidget* controlWidget, QWidget* parent = nullptr);

  private:
    StopButton* m_stopButton;
    QWidget* m_contentWidget;
};