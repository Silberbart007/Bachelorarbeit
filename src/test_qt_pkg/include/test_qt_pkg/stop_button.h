#pragma once

#include <QPushButton>

class StopButton : public QPushButton {
    Q_OBJECT

  public:
    explicit StopButton(QWidget* parent = nullptr);

  signals:
    void stopPressed();

  private slots:
    void handleClick();
};