#include "control_widget_wrapper.h"

ControlWidgetWrapper::ControlWidgetWrapper(QWidget* controlWidget, QWidget* parent)
    : QWidget(parent), m_contentWidget(controlWidget) {

    this->setObjectName("control_wrapper");
    //this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(5, 5, 5, 5);

    auto* topBar = new QHBoxLayout();
    m_stopButton = new StopButton(this);

    topBar->addWidget(m_stopButton);

    topBar->setAlignment(m_stopButton, Qt::AlignLeft | Qt::AlignTop);

    mainLayout->addLayout(topBar);
    //m_contentWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    mainLayout->addWidget(m_contentWidget);
}