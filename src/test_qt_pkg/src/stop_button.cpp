#include "stop_button.h"

StopButton::StopButton(QWidget* parent) : QPushButton("Stop", parent) {
    setStyleSheet("background-color: red; color: white;");
    connect(this, &QPushButton::clicked, this, &StopButton::handleClick);
}

void StopButton::handleClick() {
    emit stopPressed();
}