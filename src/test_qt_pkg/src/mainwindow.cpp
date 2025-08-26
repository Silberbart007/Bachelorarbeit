#include "mainwindow.h"
#include "ui_mainwindow.h"

// ===== Static variables =====
bool MainWindow::s_dynLock = true;
bool MainWindow::s_statLock = false;

// =====================
// Public Methods
// =====================

/**
 * @brief Constructs the main window and initializes all components.
 *
 * Sets up the UI, ROS nodes, callbacks, control widgets, styles,
 * synchronization timers, and initial UI states.
 *
 * @param parent Optional parent widget.
 */
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), m_ui(new Ui::MainWindow), m_parkingMode(false), m_vectorMode(false),
      m_tapControlMode(false) {
    // ===== UI Setup =====
    m_ui->setupUi(this);
    this->resize(3840, 2160); // Resize window to 4K resolution

    // ===== ROS Nodes Initialization =====
    m_robot_node = std::make_shared<RobotNode>();
    m_nav2_node = std::make_shared<Nav2Client>();

    // ===== Register Callbacks =====
    m_robot_node->on_image_received = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->image_callback(msg);
    };
    m_robot_node->on_scan_received = [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        m_ui->obstacle_map_widget->laser_callback(msg);
        m_ui->ego_widget->laser_callback(msg);
        m_ui->laser_map_widget->laser_callback(msg);
    };

    // ===== Assign Nodes to UI Widgets =====
    m_ui->rotation_slider->setRobotNode(m_robot_node);
    m_ui->speed_slider->setRobotNode(m_robot_node);
    m_ui->speed_slider_wheels->setRobotNode(m_robot_node);
    m_ui->rotation_slider_joystick->setRobotNode(m_robot_node);
    m_ui->wheels->setRobotNode(m_robot_node);
    m_ui->wheels_2->setRobotNode(m_robot_node);
    m_ui->joysticks->setRobotNode(m_robot_node);
    m_ui->rotation_joystick->setRobotNode(m_robot_node);
    m_ui->obstacle_map_widget->setRobotNode(m_robot_node);
    m_ui->obstacle_map_widget->setNav2Node(m_nav2_node);

    // ===== Configure Wheel Widget Style =====
    WheelStyle formula1Style;
    formula1Style.outerRingColor = QColor(20, 20, 20);
    formula1Style.outerRingWidth = 30;
    formula1Style.spokeColor = QColor(255, 0, 0);
    formula1Style.spokeWidth = 30;
    formula1Style.centerCircleColor = QColor(50, 0, 0);
    formula1Style.centerCircleRadius = 35;
    formula1Style.centerText = "F1";
    formula1Style.centerFont = QFont("Orbitron", 20, QFont::Bold);
    formula1Style.centerTextColor = Qt::yellow;
    formula1Style.maxAngle = 270.0;
    formula1Style.isRound = false;
    m_ui->wheels_2->setStyle(formula1Style);

    // ===== Setup Synchronization Timer =====
    QTimer* syncTimer = new QTimer(this);
    connect(syncTimer, &QTimer::timeout, this, [this]() {
        if (!m_robot_node)
            return;

        RobotNode::RobotSpeed speedNorm = m_robot_node->getSpeedNormalized();
        double rotNorm = m_robot_node->getRotationNormalized();

        m_ui->speed_slider->setValue(speedNorm.x);
        m_ui->rotation_slider->setValue(rotNorm);
        m_ui->speed_slider_wheels->setValue(speedNorm.x);
        m_ui->rotation_slider_joystick->setValue(rotNorm);
        m_ui->wheels->setValue(rotNorm);
        m_ui->wheels_2->setValue(rotNorm);
        m_ui->joysticks->setValue(speedNorm);
    });
    syncTimer->start(50); // every 50 ms

    // ===== Setup Label timers =====
    m_timer_elapsedTenthsSecond = 0;
    QTimer* laserUpdateTimer = new QTimer(this);
    connect(laserUpdateTimer, &QTimer::timeout, this, [this]() { logEvent(); });
    laserUpdateTimer->start(100);

    m_ui->timer_label->setText(QString("Timer: 00:00.00"));
    m_timer = new QTimer(this);
    m_timer->setInterval(100);
    connect(m_timer, &QTimer::timeout, this, &MainWindow::updateTimerLabel);

    // ===== Wrap all control widgets ======
    // (e.g. for displaying the default stop button)
    std::vector<QWidget**> controls = {
        &m_ui->buttons_container,    &m_ui->sliders,
        &m_ui->JoystickLayout,       &m_ui->WheelsLayout,
        &m_ui->cam_widget,           &m_ui->ObstacleMapLayout,
        &m_ui->ego_widget_container, &m_ui->laser_map_widget_container};

    for (QWidget** pw : controls) {
        QWidget* oldWidget = *pw;
        QWidget* parent = oldWidget->parentWidget();
        QLayout* layout = parent ? parent->layout() : nullptr;

        if (layout) {
            layout->removeWidget(oldWidget);
        }

        oldWidget->setParent(nullptr);

        auto* wrapper = new ControlWidgetWrapper(oldWidget, parent);

        if (layout) {
            layout->addWidget(wrapper);
            if (*pw != m_ui->cam_widget && *pw != m_ui->ObstacleMapLayout &&
                *pw != m_ui->ego_widget_container && *pw != m_ui->laser_map_widget_container)
                layout->setAlignment(wrapper, Qt::AlignHCenter);
        }

        *pw = wrapper;
    }

    // Connect all standard stop buttons to custom function
    QList<StopButton*> buttons = this->findChildren<StopButton*>();

    for (StopButton* btn : buttons) {
        btn->installEventFilter(this);
        connect(btn, &QPushButton::clicked, this, &MainWindow::on_stop_full_button_2_pressed);
    }

    // ===== Hide Optional UI Elements Initially =====
    m_ui->WheelsLayout->setVisible(false);
    m_ui->JoystickLayout->setVisible(false);
    m_ui->sliders->setVisible(false);
    m_ui->buttons_container->setVisible(false);
    m_ui->AllOptionsLayout->setVisible(false);

    // Parameter controls hidden at startup
    m_ui->curve_gain_slider->setVisible(false);
    m_ui->curve_gain_label->setVisible(false);
    m_ui->ghost_duration_slider->setVisible(false);
    m_ui->ghost_duration_label->setVisible(false);
    m_ui->ghost_color_button->setVisible(false);
    m_ui->laser_number_label->setVisible(false);
    m_ui->laser_number_slider->setVisible(false);
    m_ui->beam_color_button->setVisible(false);
    m_ui->trail_lifetime_slider->setVisible(false);
    m_ui->trail_lifetime_label->setVisible(false);
    m_ui->trail_color_button->setVisible(false);

    // ===== Initialize Parameter Labels and Sliders =====
    m_ui->curve_gain_label->setText(QString("Curve Gain: %1").arg(1.25));
    m_ui->ghost_duration_label->setText(QString("Ghost duration: %1 s").arg(2.0));
    m_ui->laser_number_label->setText(QString("Laser number: %1").arg(540));
    m_ui->trail_lifetime_label->setText(QString("Trail lifetime: %1 s").arg(2.0));
    m_ui->zoom_factor_label->setText(QString("Zoom factor: %1").arg(1.0));

    m_ui->rotation_slider->setValue(0.0);
    m_ui->speed_slider->setValue(0.0);
    m_ui->speed_slider_wheels->setValue(0.0);
    m_ui->rotation_slider_joystick->setValue(0.0);

    // ===== Initialize Mode Selection =====
    m_ui->mode_list_2->setCurrentRow(0);
    m_ui->mode_list_view->item(0)->setSelected(true);
    m_ui->mode_list_view->item(1)->setSelected(true);
    m_ui->mode_list_view->item(2)->setSelected(false);
    m_ui->lock_options_list->setCurrentRow(1);
    m_ui->lock_options_list->item(1)->setSelected(true);

    // ===== Configure Joystick types =====
    m_ui->joysticks->setOmni(true);
    m_ui->rotation_joystick->setOmni(false);

    // ===== Install Event Filters =====
    QList<QWidget*> widgetsToMonitor = {m_ui->cam_label,
                                        m_ui->joysticks,
                                        m_ui->rotation_joystick,
                                        m_ui->rotation_slider_joystick,
                                        m_ui->speed_slider_wheels,
                                        m_ui->wheels,
                                        m_ui->wheels_2,
                                        m_ui->anticlockwise_fast_button,
                                        m_ui->anticlockwise_slow_button,
                                        m_ui->fast_button,
                                        m_ui->slow_button,
                                        m_ui->stop_button,
                                        m_ui->clockwise_fast_button,
                                        m_ui->clockwise_slow_button,
                                        m_ui->reset_rotation_button,
                                        m_ui->rotation_slider,
                                        m_ui->reset_rotation_button_2,
                                        m_ui->speed_slider,
                                        m_ui->AllOptionsLayout,
                                        m_ui->modes_button,
                                        m_ui->stop_full_button_2,
                                        m_ui->start_timer_button,
                                        m_ui->stop_timer_button,
                                        m_ui->beam_color_button,
                                        m_ui->curve_gain_slider,
                                        m_ui->ghost_color_button,
                                        m_ui->ghost_duration_slider,
                                        m_ui->laser_number_slider,
                                        m_ui->follow_checkBox,
                                        m_ui->trail_color_button,
                                        m_ui->trail_lifetime_slider,
                                        m_ui->cam_label,
                                        m_ui->ego_widget_container,
                                        m_ui->laser_map_widget_container};

    for (QWidget* w : widgetsToMonitor) {
        w->installEventFilter(this);
    }

    m_ui->obstacle_map_widget->view()->viewport()->setObjectName("obstacle_map");
    m_ui->obstacle_map_list->viewport()->setObjectName("obstacle_map_list");
    m_ui->cam_list->viewport()->setObjectName("cam_list");

    m_ui->obstacle_map_widget->view()->viewport()->installEventFilter(this);
    m_ui->obstacle_map_list->viewport()->installEventFilter(this);
    m_ui->cam_list->viewport()->installEventFilter(this);

    // Configure Timer for sending velocity of buttons
    m_velocityTimer = new QTimer(this);
    connect(m_velocityTimer, &QTimer::timeout, this, &MainWindow::sendCurrentVelocity);

    // ===== Initialize logging files =====
    initLogging();

    // Fullscreen
    this->showFullScreen();
}

/**
 * @brief Destructor for MainWindow.
 *
 * Cleans up the UI pointer.
 */
MainWindow::~MainWindow() {
    if (m_laser_logFile.isOpen()) {
        m_laser_logFile.close();
    }

    delete m_ui;
}

// =====================
// Protected Methods
// =====================

/**
 * @brief Event filter to capture mouse and touch events.
 *
 * This function intercepts mouse button press events on the camera label widget.
 * If tap control mode is enabled, it calculates the normalized horizontal position
 * of the tap relative to the displayed camera image and publishes velocity commands
 * to the robot accordingly.
 *
 * @param obj The object sending the event.
 * @param event The event to be filtered.
 * @return True if the event was handled, false otherwise.
 */
bool MainWindow::eventFilter(QObject* obj, QEvent* event) {

    // Save current GUI interaction for logging reasons
    if (event->type() == QEvent::MouseButtonPress || event->type() == QEvent::TouchBegin ||
        event->type() == QEvent::TouchUpdate || event->type() == QEvent::TouchEnd) {
        QWidget* widget = qobject_cast<QWidget*>(obj);
        if (widget) {
            QString name = widget->objectName();
            if (name.isEmpty())
                name = widget->metaObject()->className();

            QPointF pos;
            QPointF globalPos;

            if (event->type() == QEvent::MouseButtonPress) {
                QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
                pos = mouseEvent->pos();
                globalPos = widget->mapTo(widget->window(), pos.toPoint());
            } else if (event->type() == QEvent::TouchBegin ||
                       event->type() == QEvent::TouchUpdate || event->type() == QEvent::TouchEnd) {
                QTouchEvent* touchEvent = static_cast<QTouchEvent*>(event);
                if (!touchEvent->touchPoints().isEmpty()) {
                    pos = touchEvent->touchPoints().first().pos();
                    globalPos = widget->mapTo(widget->window(), pos.toPoint());
                }
            }

            // In den Zwischenspeicher eintragen
            QMutexLocker locker(&m_interactionMutex);
            m_recentInteractions.append({name, globalPos});
        }
    }

    // Handle mouse button press event
    if (event->type() == QEvent::MouseButtonPress) {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        QPoint clickPos = mouseEvent->pos(); // Click position within cam_label

        // Only handle taps on the camera label when tap control mode is active
        if (obj == m_ui->cam_label && m_tapControlMode) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

            const QPixmap* pixmap = m_ui->cam_label->pixmap();
            if (!pixmap || pixmap->isNull())
                return false;

#pragma GCC diagnostic pop

            QSize labelSize = m_ui->cam_label->size();
            QSize pixmapSize = pixmap->size();
            QSize scaledSize = pixmapSize.scaled(labelSize, Qt::KeepAspectRatio);

            int offsetX = (labelSize.width() - scaledSize.width()) / 2;
            int offsetY = (labelSize.height() - scaledSize.height()) / 2;

            int relativeX = clickPos.x() - offsetX;
            int relativeY = clickPos.y() - offsetY;

            if (relativeX < 0 || relativeX >= scaledSize.width() || relativeY < 0 ||
                relativeY >= scaledSize.height())
                return false;

            // Normalize horizontal click position to range [-1.0, 1.0]
            double dx = (relativeX - scaledSize.width() / 2.0) / (scaledSize.width() / 2.0);
            dx = std::clamp(dx, -1.0, 1.0);

            double dy = static_cast<double>(clickPos.y()) / scaledSize.height();
            dy = std::clamp(dy, 0.0, 1.0);

            // Umgekehrte Logik: oben schnell, unten langsam
            float max_speed = 1.0f;
            float linear = (1.0 - dy) * max_speed;

            if (m_robot_node) {
                m_robot_node->publish_velocity({linear, 0.0}, dx);
            }

            return true;
        }
    }
    // Default event processing
    return QMainWindow::eventFilter(obj, event);
}

// =====================
// Private Slot Methods
// =====================

/**
 * @brief Handles changes in the selection of the control modes list.
 *
 * This slot is triggered when the selection in the control mode list changes.
 * It checks which control modes are selected and shows or hides the corresponding UI widgets
 * accordingly.
 */
void MainWindow::on_mode_list_2_itemSelectionChanged() {
    // Get all currently selected items in the mode list
    QList<QListWidgetItem*> selectedItems = m_ui->mode_list_2->selectedItems();

    // Flags to indicate which control widgets should be shown
    bool showWheel = false;
    bool defaultWheel = false;
    bool raceWheel = false;
    bool showJoystick = false;
    bool showButtons = false;
    bool showSliders = false;
    bool showRotationJoystick = false;

    // Evaluate selected items and set flags
    for (QListWidgetItem* item : selectedItems) {
        QString text = item->text();
        if (text == "Default Lenkrad") {
            showWheel = true;
            defaultWheel = true;
        } else if (text == "Rennauto Lenkrad") {
            showWheel = true;
            raceWheel = true;
        } else if (text == "Omni-Joystick") {
            showJoystick = true;
        } else if (text == "Buttons") {
            showButtons = true;
        } else if (text == "Slider") {
            showSliders = true;
        } else if (text == "Rotation-Joystick") {
            showRotationJoystick = true;
        }
    }

    // Show or hide the control widgets based on selection
    m_ui->WheelsLayout->setVisible(showWheel);
    m_ui->wheels->setVisible(defaultWheel);
    m_ui->wheels_2->setVisible(raceWheel);
    m_ui->JoystickLayout->setVisible(showJoystick | showRotationJoystick);
    m_ui->joysticks->setVisible(showJoystick);
    m_ui->rotation_slider_joystick->setVisible(showJoystick);
    m_ui->rotation_joystick->setVisible(showRotationJoystick);
    m_ui->sliders->setVisible(showSliders);
    m_ui->buttons_container->setVisible(showButtons);

    update();
}

/**
 * @brief Handles changes in the selection of the display options list.
 *
 * This slot is called when the selection in the display mode list changes.
 * It toggles visibility of the camera and map widgets based on the current selection.
 */
void MainWindow::on_mode_list_view_itemSelectionChanged() {
    // Get all currently selected items in the display options list
    QList<QListWidgetItem*> selectedItems = m_ui->mode_list_view->selectedItems();

    // Flags to determine which display widgets to show
    bool showCam = false;
    bool showMap = false;
    bool showEgo = false;
    bool showLaserMap = false;

    // Check selected items and set visibility flags
    for (QListWidgetItem* item : selectedItems) {
        QString text = item->text();
        if (text == "Camera") {
            showCam = true;
        } else if (text == "Map") {
            showMap = true;
        } else if (text == "Ego") {
            showEgo = true;
        } else if (text == "Laser Map") {
            showLaserMap = true;
        }
    }

    // Show or hide display widgets accordingly
    m_ui->cam_widget->setVisible(showCam);
    m_ui->ObstacleMapLayout->setVisible(showMap);
    m_ui->ego_widget_container->setVisible(showEgo);
    m_ui->laser_map_widget_container->setVisible(showLaserMap);

    m_ui->cam_label->clear();
    m_ui->cam_label->updateGeometry();

    update();
}

/**
 * @brief Handles changes in the selection of the lock options list.
 *
 * This slot is called when the selection in the lock options list changes.
 * It toggles the lock options for all active sliders based on the current selection.
 */
void MainWindow::on_lock_options_list_itemSelectionChanged() {
    // Get all currently selected items in the display options list
    QList<QListWidgetItem*> selectedItems = m_ui->lock_options_list->selectedItems();

    // Flags to determine which display widgets to show
    bool dynLock = false;
    bool statLock = false;

    // Check selected items and set visibility flags
    for (QListWidgetItem* item : selectedItems) {
        QString text = item->text();
        if (text == "Dynamic lock") {
            dynLock = true;
        } else if (text == "Always lock") {
            statLock = true;
        }
    }

    // Toggle static variables accordingly
    MainWindow::s_dynLock = dynLock;
    MainWindow::s_statLock = statLock;

    update();
}

/**
 * @brief Toggles the visibility of the options panel.
 *
 * This slot is called when the "Modes" button is clicked.
 * It shows or hides the entire options layout.
 */
void MainWindow::on_modes_button_clicked() {
    // Toggle the visibility of the options panel
    m_ui->AllOptionsLayout->setVisible(!m_ui->AllOptionsLayout->isVisible());
}

/**
 * @brief Sends a fast forward velocity command to the robot.
 */
void MainWindow::on_fast_button_pressed() {
    m_button_linear_x = 1.0;
    m_button_linear_y = 0.0;
    m_button_angular_z = m_robot_node->getRotationNormalized();

    sendCurrentVelocity();
    m_velocityTimer->start(50);
}

void MainWindow::on_fast_button_released() {
    m_velocityTimer->stop();
}

/**
 * @brief Sends a slow forward velocity command to the robot.
 */
void MainWindow::on_slow_button_pressed() {
    m_button_linear_x = 0.5;
    m_button_linear_y = 0.0;
    m_button_angular_z = m_robot_node->getRotationNormalized();

    sendCurrentVelocity();
    m_velocityTimer->start(50);
}

void MainWindow::on_slow_button_released() {
    m_velocityTimer->stop();
}

/**
 * @brief Sends a stop command to the robot (zero velocity).
 */
void MainWindow::on_stop_button_pressed() {
    m_button_linear_x = 0.0;
    m_button_linear_y = 0.0;
    m_button_angular_z = m_robot_node->getRotationNormalized();

    sendCurrentVelocity();
    m_velocityTimer->start(50);
}

/**
 * @brief Sends a full stop command to the robot (zero velocity AND rotation AND stop
 * followPath/Pose-Client).
 */
void MainWindow::on_stop_full_button_2_pressed() {
    m_velocityTimer->stop();
    m_nav2_node->cancelGoalsPose();
    m_nav2_node->cancelGoalsFollow();
    m_ui->obstacle_map_widget->stopInertia();
    m_ui->obstacle_map_widget->abortJakobDrive();
    m_robot_node->publish_velocity({0.0, 0.0}, 0.0);

    // Button Velocities reset
    m_button_linear_x = 0.0;
    m_button_angular_z = 0.0;
    m_button_linear_y = 0.0;
    sendCurrentVelocity();
}

/**
 * @brief Sends a slow backward velocity command to the robot.
 */
void MainWindow::on_back_slow_button_pressed() {
    m_button_linear_x = -0.5;
    m_button_linear_y = 0.0;
    m_button_angular_z = m_robot_node->getRotationNormalized();

    sendCurrentVelocity();
    m_velocityTimer->start(50);
}

void MainWindow::on_back_slow_button_released() {
    m_velocityTimer->stop();
}

/**
 * @brief Sends a fast backward velocity command to the robot.
 */
void MainWindow::on_back_fast_button_pressed() {
    m_button_linear_x = -1.0;
    m_button_linear_y = 0.0;
    m_button_angular_z = m_robot_node->getRotationNormalized();

    sendCurrentVelocity();
    m_velocityTimer->start(50);
}

void MainWindow::on_back_fast_button_released() {
    m_velocityTimer->stop();
}

/**
 * @brief Sends a slow clockwise rotation velocity command to the robot.
 */
void MainWindow::on_clockwise_slow_button_pressed() {
    m_button_angular_z = 0.5;

    sendCurrentVelocity();
    m_velocityTimer->start(50);
}

void MainWindow::on_clockwise_slow_button_released() {
    m_velocityTimer->stop();
}

/**
 * @brief Sends a fast clockwise rotation velocity command to the robot.
 */
void MainWindow::on_clockwise_fast_button_pressed() {
    m_button_angular_z = 1.0;

    sendCurrentVelocity();
    m_velocityTimer->start(50);
}

void MainWindow::on_clockwise_fast_button_released() {
    m_velocityTimer->stop();
}

/**
 * @brief Sends a slow anticlockwise rotation velocity command to the robot.
 */
void MainWindow::on_anticlockwise_slow_button_pressed() {
    m_button_angular_z = -0.5;

    sendCurrentVelocity();
    m_velocityTimer->start(50);
}

void MainWindow::on_anticlockwise_slow_button_released() {
    m_velocityTimer->stop();
}

/**
 * @brief Sends a fast anticlockwise rotation velocity command to the robot.
 */
void MainWindow::on_anticlockwise_fast_button_pressed() {
    m_button_angular_z = -1.0;

    sendCurrentVelocity();
    m_velocityTimer->start(50);
}

void MainWindow::on_anticlockwise_fast_button_released() {
    m_velocityTimer->stop();
}

/**
 * @brief Resets the robot's rotation by setting angular velocity to zero.
 *
 * Keeps the current linear speed unchanged.
 */
void MainWindow::on_reset_rotation_button_pressed() {
    m_button_angular_z = 0.0;
    sendCurrentVelocity();
}

/**
 * @brief Resets the robot's rotation by setting angular velocity to zero.
 *
 * Keeps the current linear speed unchanged.
 */
void MainWindow::on_reset_rotation_button_2_pressed() {
    m_button_angular_z = 0.0;
    sendCurrentVelocity();
}

/**
 * @brief Starts timer
 */
void MainWindow::on_start_timer_button_clicked() {
    startTimer();
}

/**
 * @brief Stops timer
 */
void MainWindow::on_stop_timer_button_clicked() {
    stopTimer();
}

/**
 * @brief Handles selection changes in the obstacle map options list.
 *
 * Updates the modes of the obstacle map widget and shows/hides
 * related parameter controls based on the selected options.
 */
void MainWindow::on_obstacle_map_list_itemSelectionChanged() {
    // Get all currently selected items
    QList<QListWidgetItem*> selectedItems = m_ui->obstacle_map_list->selectedItems();

    // Flags to indicate which obstacle map modes should be active
    bool drawPathMode = false;
    bool beamMode = false;
    bool followMode = false;
    bool ghostMode = false;
    bool inertiaMode = false;
    bool trailMode = false;
    bool collisionBorderMode = false;
    bool zoneMode = false;
    bool jakobMode = false;

    // Check which items are selected and set the corresponding flags
    for (QListWidgetItem* item : selectedItems) {
        QString text = item->text();
        if (text == "Draw path") {
            drawPathMode = true;
        } else if (text == "Beam") {
            beamMode = true;
        } else if (text == "Follow Finger") {
            followMode = true;
        } else if (text == "Ghost Robot") {
            ghostMode = true;
        } else if (text == "Inertia-Control") {
            inertiaMode = true;
        } else if (text == "Trail") {
            trailMode = true;
        } else if (text == "Collision Warning (border)") {
            collisionBorderMode = true;
        } else if (text == "Zone Control") {
            zoneMode = true;
        } else if (text == "Jakob") {
            jakobMode = true;
        }
    }

    // Update obstacle map widget modes accordingly
    m_ui->obstacle_map_widget->setDrawPathMode(drawPathMode);
    m_ui->obstacle_map_widget->setBeamMode(beamMode);
    m_ui->obstacle_map_widget->setFollowMode(followMode);
    m_ui->obstacle_map_widget->setGhostMode(ghostMode);
    m_ui->obstacle_map_widget->setInertiaMode(inertiaMode);
    m_ui->obstacle_map_widget->setTrailMode(trailMode);
    m_ui->obstacle_map_widget->setCollisionBorderMode(collisionBorderMode);
    m_ui->obstacle_map_widget->setZoneMode(zoneMode);
    m_ui->obstacle_map_widget->setJakobMode(jakobMode);

    // Show or hide parameter controls based on active modes
    m_ui->curve_gain_slider->setVisible(ghostMode);
    m_ui->curve_gain_label->setVisible(ghostMode);
    m_ui->ghost_duration_slider->setVisible(ghostMode);
    m_ui->ghost_duration_label->setVisible(ghostMode);
    m_ui->ghost_color_button->setVisible(ghostMode);
    m_ui->beam_color_button->setVisible(beamMode);
    m_ui->laser_number_label->setVisible(beamMode);
    m_ui->laser_number_slider->setVisible(beamMode);
    m_ui->trail_color_button->setVisible(trailMode);
    m_ui->trail_lifetime_label->setVisible(trailMode);
    m_ui->trail_lifetime_slider->setVisible(trailMode);
}

/**
 * @brief Handles selection changes in the camera mode options list.
 *
 * Updates the enabled camera assistance modes based on the selected options.
 */
void MainWindow::on_cam_list_itemSelectionChanged() {
    // Get all currently selected items
    QList<QListWidgetItem*> selectedItems = m_ui->cam_list->selectedItems();

    // Reset all camera mode flags
    m_vectorMode = false;
    m_parkingMode = false;
    m_tapControlMode = false;

    // Set flags based on selected items
    for (QListWidgetItem* item : selectedItems) {
        QString text = item->text();
        if (text == "Vector arrow") {
            m_vectorMode = true;
        } else if (text == "Parking mode") {
            m_parkingMode = true;
        } else if (text == "Tap-Control") {
            m_tapControlMode = true;
        }
    }
}

/**
 * @brief Called when the curve gain slider value changes.
 *
 * Updates the curve gain parameter in the obstacle map widget and
 * updates the corresponding label text.
 *
 * @param value The new slider value (0-100 expected).
 */
void MainWindow::on_curve_gain_slider_valueChanged(int value) {
    m_ui->obstacle_map_widget->setCurveGain(static_cast<double>(value) / 100.0);
    m_ui->curve_gain_label->setText(QString("Curve Gain: %1").arg(value));
}

/**
 * @brief Called when the ghost duration slider value changes.
 *
 * Updates the ghost duration parameter in the obstacle map widget and
 * updates the corresponding label text.
 *
 * @param value The new slider value (duration in seconds).
 */
void MainWindow::on_ghost_duration_slider_valueChanged(int value) {
    m_ui->obstacle_map_widget->setGhostDuration(static_cast<double>(value));
    m_ui->ghost_duration_label->setText(QString("Ghost duration: %1 s").arg(value));
}

/**
 * @brief Called when the laser number slider value changes.
 *
 * Updates the number of laser beams in the obstacle map widget and
 * updates the corresponding label text.
 *
 * @param value The new laser number.
 */
void MainWindow::on_laser_number_slider_valueChanged(int value) {
    m_ui->obstacle_map_widget->setLaserNumber(value);
    m_ui->laser_number_label->setText(QString("Laser number: %1").arg(value));
}

/**
 * @brief Called when the zoom factor slider value changes.
 *
 * Updates the zoom factor of Ego view
 *
 * @param value The new zoom factor.
 */
void MainWindow::on_zoom_factor_slider_valueChanged(int value) {
    m_ui->ego_widget->setZoomFactor(static_cast<double>(value) / 100.0);
    m_ui->zoom_factor_label->setText(
        QString("Zoom factor: %1").arg(m_ui->ego_widget->getZoomFactor()));
}

/**
 * @brief Slot called when the trail lifetime slider value changes.
 *
 * Maps the linear slider value to an exponential scale to allow fine
 * adjustments at low values (up to ~10 seconds) and faster increase at higher
 * values (up to 10 minutes). Updates the trail lifetime parameter in the obstacle
 * map widget and the corresponding label.
 *
 * @param value The new slider value (expected range: 0 to 100).
 */
void MainWindow::on_trail_lifetime_slider_valueChanged(int value) {
    const double minSeconds = 0.1;   ///< Minimum trail lifetime in seconds.
    const double maxSeconds = 600.0; ///< Maximum trail lifetime in seconds (10 minutes).

    // Normalize slider value to [0,1]
    double t = static_cast<double>(value) / 100.0;

    // Exponential scaling formula:
    // lifetimeSeconds = minSeconds * (maxSeconds / minSeconds)^t
    double lifetimeSeconds = minSeconds * std::pow(maxSeconds / minSeconds, t);

    // clamp very small values to zero for better UX
    if (lifetimeSeconds < 0.1) {
        lifetimeSeconds = 0.0;
    }

    // Update the trail lifetime in the obstacle map widget
    m_ui->obstacle_map_widget->setTrailLifetime(lifetimeSeconds);

    // Format the label: display seconds under 60, otherwise minutes
    QString labelText;
    if (lifetimeSeconds < 60.0) {
        labelText = QString("Trail lifetime: %1 s").arg(lifetimeSeconds, 0, 'f', 2);
    } else {
        double minutes = lifetimeSeconds / 60.0;
        labelText = QString("Trail lifetime: %1 min").arg(minutes, 0, 'f', 2);
    }
    m_ui->trail_lifetime_label->setText(labelText);
}

/**
 * @brief Slot for the Beam Color button click.
 *
 * Opens a color picker dialog to select the laser beam color.
 * Updates the button background and notifies the obstacle map widget.
 */
void MainWindow::on_beam_color_button_clicked() {
    QColor color = QColorDialog::getColor(Qt::red, this, "Choose Color");

    if (color.isValid()) {
        qDebug() << "Selected color:" << color;

        // Change button background color
        QString qss = QString("background-color: %1").arg(color.name());
        m_ui->beam_color_button->setStyleSheet(qss);

        // Send color to obstacle map widget
        m_ui->obstacle_map_widget->setLaserColor(color);
    }
}

/**
 * @brief Slot for the Trail Color button click.
 *
 * Opens a color picker dialog to select the trail color.
 * Updates the button background and notifies the obstacle map widget.
 */
void MainWindow::on_trail_color_button_clicked() {
    QColor color = QColorDialog::getColor(Qt::red, this, "Choose Color");

    if (color.isValid()) {
        qDebug() << "Selected color:" << color;

        QString qss = QString("background-color: %1").arg(color.name());
        m_ui->trail_color_button->setStyleSheet(qss);

        m_ui->obstacle_map_widget->setTrailColor(color);
    }
}

/**
 * @brief Slot for the Ghost Color button click.
 *
 * Opens a color picker dialog to select the ghost display color.
 * Updates the button background and notifies the obstacle map widget.
 */
void MainWindow::on_ghost_color_button_clicked() {
    QColor color = QColorDialog::getColor(Qt::red, this, "Choose Color");

    if (color.isValid()) {
        qDebug() << "Selected color:" << color;

        QString qss = QString("background-color: %1").arg(color.name());
        m_ui->ghost_color_button->setStyleSheet(qss);

        m_ui->obstacle_map_widget->setGhostColor(color);
    }
}

/**
 * @brief Projects a 3D point onto a 2D image plane using intrinsic camera parameters.
 *
 * This function uses the pinhole camera model to convert 3D coordinates (x, y, z)
 * into 2D pixel coordinates on the image.
 *
 * @param x X coordinate in 3D space.
 * @param y Y coordinate in 3D space.
 * @param z Z coordinate (depth) in 3D space. Must be non-zero.
 * @param fx Focal length in x direction (pixels).
 * @param fy Focal length in y direction (pixels).
 * @param cx Principal point offset in x direction (pixels).
 * @param cy Principal point offset in y direction (pixels).
 * @return cv::Point The projected 2D point in image coordinates.
 */
cv::Point MainWindow::projectToImage(double x, double y, double z, double fx, double fy, double cx,
                                     double cy) {
    int img_x = static_cast<int>(fx * x / z + cx);
    int img_y = static_cast<int>(fy * y / z + cy);
    return cv::Point(img_x, img_y);
}

/**
 * @brief Slot for the follow check Box
 *
 * Activates follow robot position if checkbox is
 * checked.
 */
void MainWindow::on_follow_checkBox_stateChanged(int state) {
    if (state == Qt::Checked) {
        m_ui->obstacle_map_widget->setMapFollow(true);
    } else if (state == Qt::Unchecked) {
        m_ui->obstacle_map_widget->setMapFollow(false);
    }
}

// =====================
// Private Methods
// =====================

/**
 * @brief Initialize logging files
 */
void MainWindow::initLogging() {
    QString timestamp = QDateTime::currentDateTime().toString(Qt::ISODate);
    QString filename = QString("custom_log/log_%1.csv").arg(timestamp);
    m_laser_logFile.setFileName(filename);
    if (m_laser_logFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        m_laser_logStream.setDevice(&m_laser_logFile);
        m_laser_logStream
            << "Timestamp,MinDistance [m],AvgDistance [m],Timer [min:s.100ms],linear velocity (x)"
               "[m/s],linear velocity"
               "(y) [m/s],angular velocity (z) [m/s],Robot amcl-position (x)"
               "[m],Robot amcl-position (y) [m],Robot amcl-theta (x) "
               "[rad],Interaction names,Interaction positions [pixels],Map Pos (x),"
               "Map Pos (y),Map Rot (Deg),Map zoom (Factor)\n";
    } else {
        qWarning() << "Cannot open logfile!";
    }
}

/**
 * @brief Start timer
 */
void MainWindow::startTimer() {
    if (m_timer_elapsedTenthsSecond > 0) {
        m_timer->stop();
        m_timer_elapsedTenthsSecond = -1;
        updateTimerLabel();
    } else {
        m_timer->start();
    }
}

/**
 * @brief Stop timer
 */
void MainWindow::stopTimer() {
    if (m_timer->isActive())
        m_timer->stop();
    else
        m_timer->start();
}

/**
 * @brief Update timer label
 */
void MainWindow::updateTimerLabel() {
    m_timer_elapsedTenthsSecond++;

    int totalSeconds = m_timer_elapsedTenthsSecond / 10;
    int minutes = totalSeconds / 60;
    int seconds = totalSeconds % 60;
    int tenths = m_timer_elapsedTenthsSecond % 10;

    m_ui->timer_label->setText(QString("Timer: %1:%2.%3")
                                   .arg(minutes, 2, 10, QChar('0'))
                                   .arg(seconds, 2, 10, QChar('0'))
                                   .arg(tenths));
}

/**
 * @brief Callback for receiving camera image data.
 *
 * This function converts the incoming ROS image message to OpenCV format,
 * then overlays visualization graphics such as the velocity vector arrow
 * and parking assist lane lines depending on the current mode.
 * Finally, it updates the Qt label with the processed image.
 *
 * @param msg Shared pointer to the ROS sensor_msgs::msg::Image message.
 */
void MainWindow::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // Convert ROS image to OpenCV BGR image
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        if (!frame.empty()) {
            // Get normalized robot speed and rotation (-1.0 to 1.0)
            RobotNode::RobotSpeed speed_value = m_robot_node->getSpeedNormalized();
            double rotation_value = m_robot_node->getRotationNormalized();

            // Mirror rotation when driving backward
            if (speed_value.x < 0.0) {
                rotation_value = -rotation_value;
            }

            // Compute arrow angle in degrees: 0° = right, 90° = up, 180° = left, 270° = down
            double rotation = 270.0 - rotation_value * 90.0;

            // Arrow start point near bottom-center of image
            cv::Point start((frame.cols - 20) / 2, frame.rows - 100);

            // Convert angle to radians
            double angle_rad = rotation * CV_PI / 180.0;

            // Direction unit vector (image Y axis points downward)
            cv::Point2f dir(-std::cos(angle_rad), std::sin(angle_rad));

            // Max arrow length in pixels
            double max_length = 120.0;

            // Scale arrow length by speed
            double length = speed_value.x * max_length;

            // Calculate arrow end point
            cv::Point end = start + cv::Point(dir.x * length, dir.y * length);

            // Arrow color: Blue for forward, Red for backward
            cv::Scalar arrow_color =
                (speed_value.x >= 0.0) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);

            // Draw arrow if speed is significant and vector mode is enabled
            if (std::abs(speed_value.x) > 0.01 && m_vectorMode) {
                int thickness = 6;
                cv::arrowedLine(frame, start, end, arrow_color, thickness);
            }

            // === Parking assist lane lines visualization ===
            if (m_parkingMode) {
                double wheel_base = 10.0;             // Distance between front and rear wheels (cm)
                double max_steering_angle = M_PI / 4; // Max steering angle (45°)
                double steering_angle = rotation_value * max_steering_angle;

                // Camera intrinsic parameters (simplified)
                double fx = 200.0;
                double fy = 200.0;
                double cx = frame.cols / 2.0;
                double cy = frame.rows / 2.0;

                double camera_height = 60.0; // Camera height in cm

                // Max lane line length in depth direction, scales with speed
                double max_z = std::min(150.0, 60.0 + std::abs(speed_value.x) * 100);
                // Always same length
                max_z = 300.0;

                int num_points = 30;  // Number of points to generate lane lines
                double offset = 25.0; // Offset between left/right lane lines in cm

                std::vector<cv::Point> left_pts;
                std::vector<cv::Point> right_pts;
                std::vector<cv::Point2f> base;

                // Generate base points along the lane center curve
                for (int i = 0; i < num_points; ++i) {
                    double z = (i / (double)num_points) * max_z;
                    double x = 0.0;

                    if (std::abs(steering_angle) > 1e-3) {
                        double R = wheel_base / std::tan(steering_angle);
                        double theta = z / R;
                        x = R * std::sin(theta);
                        z = R * (1 - std::cos(theta));
                    } else {
                        // Straight line if steering near zero
                        z = 0.0;
                        x = (i / (double)num_points) * max_z;
                    }

                    base.emplace_back(x, z);
                }

                double y_cam = camera_height;

                // Compute left and right lane line points offset perpendicular to center curve
                for (auto& b : base) {
                    double len = hypot(b.x, b.y);
                    double perp_z = b.y / len;
                    double perp_x = -b.x / len;

                    cv::Point2f L(b.y + perp_x * offset, b.x + perp_z * offset);
                    cv::Point2f R(b.y - perp_x * offset, b.x - perp_z * offset);

                    if (L.y > 0)
                        left_pts.emplace_back(projectToImage(L.x, y_cam, L.y, fx, fy, cx, cy));
                    if (R.y > 0)
                        right_pts.emplace_back(projectToImage(R.x, y_cam, R.y, fx, fy, cx, cy));
                }

                // Draw lane lines in green
                cv::polylines(frame, left_pts, false, cv::Scalar(0, 255, 0), 3);
                cv::polylines(frame, right_pts, false, cv::Scalar(0, 255, 0), 3);
            }
            // === End parking assist ===

            // Convert BGR to RGB for Qt
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

            // Convert OpenCV Mat to QImage
            QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
            QPixmap pixmap = QPixmap::fromImage(qimg);

            // Scale pixmap to label size keeping aspect ratio
            QPixmap scaledPixmap =
                pixmap.scaled(m_ui->cam_label->size(), Qt::KeepAspectRatio, Qt::FastTransformation);

            // Update QLabel pixmap in the Qt main thread
            QMetaObject::invokeMethod(
                this, [this, scaledPixmap]() { m_ui->cam_label->setPixmap(scaledPixmap); },
                Qt::QueuedConnection);
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(m_robot_node->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void MainWindow::logEvent() {
    float min = m_ui->obstacle_map_widget->getMinLaserDistance();
    float avgDist = m_ui->obstacle_map_widget->getAvgLaserDistance();
    QString timer_text = m_ui->timer_label->text();
    QString only_time_text = timer_text.mid(7);
    geometry_msgs::msg::Twist vel = m_robot_node->getLastCmdVel();
    ObstacleMapWidget::Pose2D robot_pos = m_ui->obstacle_map_widget->getRobotPositionMeters();

    ObstacleMapWidget::ViewData view = m_ui->obstacle_map_widget->getViewData();

    m_ui->laser_distance_label->setText("Smallest distance: " + QString::number(min, 'f', 2) +
                                        " m");

    // Information about current interactions
    QString interactionNames;
    QString interactionPositions;

    {
        QMutexLocker locker(&m_interactionMutex);
        for (const auto& inter : m_recentInteractions) {

            interactionNames += inter.widgetName + "|";

            interactionPositions += QString("(%1/%2)|")
                                        .arg(inter.position.x(), 0, 'f', 0)
                                        .arg(inter.position.y(), 0, 'f', 0);
        }
        m_recentInteractions.clear();
    }

    // Write in logfile
    if (m_laser_logFile.isOpen()) {
        m_laser_logStream << QDateTime::currentDateTime().toString(Qt::ISODate) << "," << min << ","
                          << avgDist << "," << only_time_text << "," << vel.linear.x << ","
                          << vel.linear.y << "," << vel.angular.z << "," << robot_pos.x << ","
                          << robot_pos.y << "," << robot_pos.theta << "," << interactionNames << ","
                          << interactionPositions << "," << view.pos.x() << "," << view.pos.y()
                          << "," << view.rot << "," << view.zoom << "\n";
        m_laser_logStream.flush();
    }
}

// For handling buttons
struct AngularState {
    bool clock_slow = false;
    bool clock_fast = false;
    bool anticlock_slow = false;
    bool anticlock_fast = false;
};

struct LinearState {
    bool forward_slow = false;
    bool forward_fast = false;
    bool back_slow = false;
    bool back_fast = false;
};

// Support Method
void updateAngularState(AngularState& state, double value) {
    switch (int(value * 10)) {
    case 5:
        state.clock_slow = true;
        break;
    case 10:
        state.clock_fast = true;
        break;
    case -5:
        state.anticlock_slow = true;
        break;
    case -10:
        state.anticlock_fast = true;
        break;
    }
}

void updateLinearState(LinearState& state, double value) {
    switch (int(value * 10)) {
    case 5:
        state.forward_slow = true;
        break;
    case 10:
        state.forward_fast = true;
        break;
    case -5:
        state.back_slow = true;
        break;
    case -10:
        state.back_fast = true;
        break;
    }
}

// Support method to set Button Style
void setButtonStyle(QPushButton* button, bool condition, const QString& qss) {
    button->setStyleSheet(condition ? qss : "");
}

// Sending current Velocity of buttons
void MainWindow::sendCurrentVelocity() {

    // Change style based on current activated button speeds
    AngularState angular;
    LinearState linear;

    updateAngularState(angular, m_button_angular_z);
    updateLinearState(linear, m_button_linear_x);

    QColor color = QColor(QColorConstants::Svg::orange);
    QString qss = QString("background-color: %1").arg(color.name());

    std::vector<std::tuple<QPushButton*, bool>> buttons = {
        {m_ui->clockwise_slow_button, angular.clock_slow},
        {m_ui->clockwise_fast_button, angular.clock_fast},
        {m_ui->anticlockwise_fast_button, angular.anticlock_fast},
        {m_ui->anticlockwise_slow_button, angular.anticlock_slow},
        {m_ui->slow_button, linear.forward_slow},
        {m_ui->fast_button, linear.forward_fast},
        {m_ui->back_slow_button, linear.back_slow},
        {m_ui->back_fast_button, linear.back_fast},
    };

    for (const auto& [button, condition] : buttons) {
        setButtonStyle(button, condition, qss);
    }

    m_robot_node->publish_velocity({m_button_linear_x, m_button_linear_y}, m_button_angular_z);
}
