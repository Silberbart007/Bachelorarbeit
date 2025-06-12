#include "mainwindow.h"
#include <QDebug>
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    {
    // Variablen Initialisieren
    m_counter = 0;
    m_parkingMode = false;
    m_vectorMode = false;
    m_tapControlMode = false;

    // Robot Node erstellen
    m_robot_node = std::make_shared<RobotNode>();

    // Nav2Client erstellen
    m_nav2_node = std::make_shared<Nav2Client>();

    // Kamera-Callback registrieren
    m_robot_node->on_image_received = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->image_callback(msg);  // ruft Funktion von MainWindow auf
    };

    // Scan-Callback registrieren
    m_robot_node->on_scan_received = [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        ui->obstacle_map_widget->laser_callback(msg); //Lasercallback aus ObstacleWidget
    };

    // UI "aktivieren"
    ui->setupUi(this);

    // robot Node an nötige Widgets übergeben
    ui->rotation_slider->setRobotNode(m_robot_node);
    ui->speed_slider->setRobotNode(m_robot_node);
    ui->wheels->setRobotNode(m_robot_node);
    ui->wheels_2->setRobotNode(m_robot_node);
    ui->speed_slider_wheels->setRobotNode(m_robot_node);
    ui->rotation_slider_joystick->setRobotNode(m_robot_node);
    ui->joysticks->setRobotNode(m_robot_node);
    ui->obstacle_map_widget->setRobotNode(m_robot_node);

    // Nav2 Client übergeben
    ui->obstacle_map_widget->setNav2Node(m_nav2_node);

    // Lenkräder Styles bestimmen
    //
    WheelStyle formula1Style;
    formula1Style.outerRingColor = QColor(20, 20, 20);  // Noch tieferes Schwarz
    formula1Style.outerRingWidth = 30;                  // Kräftiger Rahmen
    formula1Style.spokeColor = QColor(255, 0, 0);        // Knalliges Rennrot
    formula1Style.spokeWidth = 30;                      // Sehr markante Speichen
    formula1Style.centerCircleColor = QColor(50, 0, 0); // Leicht rötlicher Mittelkreis
    formula1Style.centerCircleRadius = 35;              // Kompakter
    formula1Style.centerText = "F1";                    // Oder Icon als Bild laden
    formula1Style.centerFont = QFont("Orbitron", 20, QFont::Bold); 
    formula1Style.centerTextColor = Qt::yellow;
    formula1Style.maxAngle = 270.0;                     
    formula1Style.isRound = false;
    ui->wheels_2->setStyle(formula1Style);
    // Ende Lenkräder Styles
    //

    // Synchronisierung von Steuerungsinterfaces durch timer (Damit z.b. Slider auf aktuelle speed gesetzt werden immer)
    QTimer* syncTimer = new QTimer(this);
    connect(syncTimer, &QTimer::timeout, this, [=]() {
    if (m_robot_node) {
        RobotNode::RobotSpeed speedNorm = m_robot_node->getSpeedNormalized();
        double rotNorm = m_robot_node->getRotationNormalized();

        // Werte im Bereich [-1, 1]
        ui->speed_slider->setValue(speedNorm.x);
        ui->rotation_slider->setValue(rotNorm);
        ui->speed_slider_wheels->setValue(speedNorm.x);
        ui->rotation_slider_joystick->setValue(rotNorm);
        ui->wheels->setValue(rotNorm);
        ui->wheels_2->setValue(rotNorm);
        ui->joysticks->setValue(speedNorm);
    }
    });
    syncTimer->start(50); // alle 50 ms aktualisieren

    // Steuerungen verstecken
    //ui->wheels->setVisible(false);
    //ui->speed_slider_wheels->setVisible(false);
    ui->WheelsLayout->setVisible(false);
    ui->JoystickLayout->setVisible(false);
    ui->sliders->setVisible(false);
    ui->ButtonsLayoutHorizontal->setVisible(false);

    // Optionsmenü verstecken
    ui->AllOptionsLayout->setVisible(false);

    // Parametereinstellungen verstecken
    ui->curve_gain_slider->setVisible(false);
    ui->curve_gain_label->setVisible(false);
    ui->ghost_duration_slider->setVisible(false);
    ui->ghost_duration_label->setVisible(false);
    ui->laser_number_label->setVisible(false);
    ui->laser_number_slider->setVisible(false);
    ui->beam_color_button->setVisible(false);
    ui->curve_gain_label->setText(QString("Curve Gain: %1").arg(1.25));
    ui->ghost_duration_label->setText(QString("Ghost duration: %1").arg(1.0));
    ui->laser_number_label->setText(QString("Laser number: %1").arg(270));

    // Hinderniskarte verstecken
    //ui->obstacle_map_widget->setVisible(false);

    // Slider Default Werte
    ui->rotation_slider->setValue(0.0);
    ui->speed_slider->setValue(0.0);
    ui->speed_slider_wheels->setValue(0.0);
    ui->rotation_slider_joystick->setValue(0.0);

    // Erstes Element standardmäßig auswählen
    ui->mode_list->setCurrentRow(0); // Wählt das erste Item aus
    ui->mode_list_view->item(0)->setSelected(true);
    ui->mode_list_view->item(1)->setSelected(true);

    // Cursor verstecken wegen Touch-Display
    //QCursor cursor(Qt::BlankCursor);
    //QApplication::setOverrideCursor(cursor);
    //QApplication::changeOverrideCursor(cursor);

    ui->cam_label->installEventFilter(this);

    // Auf vollen Bildschirm (4k) aktivieren
    this->resize(3840, 2160);
}

MainWindow::~MainWindow()
{
    delete ui;
}

// Maus/Touch abfangen
bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == ui->cam_label && event->type() == QEvent::MouseButtonPress && m_tapControlMode)
    {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        QPoint clickPos = mouseEvent->pos();  // Klickposition im cam_label

        int img_width = ui->cam_label->width();
        int img_height = ui->cam_label->height();

        // Berechne Richtung vom Mittelpunkt aus
        float dx = clickPos.x() - img_width / 2;
        float dy = img_height / 2 - clickPos.y();  // Y-Achse invertiert
        float angle = std::atan2(dy, dx);  // [-π, π]

        qDebug() << "Tapped at: " << clickPos << ", angle=" << angle;
        
        // Roboter steuern – je nach API
        if (m_robot_node) {
            float linear = 0.2;   // m/s
            // Normiert:
            float angular = angle / M_PI;              // [-1, 1]
            angular = std::clamp(angular, -1.0f, 1.0f);
            m_robot_node->publish_velocity({linear,0.0}, angular);  // oder dein Kommando
        }

        return true;  // Event wurde behandelt
    }
    return QMainWindow::eventFilter(obj, event);
}


// Hilfsfunktion zum projezieren eines 3D Objekts auf dem 2D Bild
cv::Point MainWindow::projectToImage(double x, double y, double z,
                         double fx, double fy,
                         double cx, double cy) {
    int img_x = static_cast<int>(fx * x / z + cx);
    int img_y = static_cast<int>(fy * y / z + cy);
    return cv::Point(img_x, img_y);
}

// Callback für Kameradaten (Subscriber)
void MainWindow::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (!frame.empty()) {

            // Aktuelle Steuerwerte holen (-1.0 bis 1.0)
            RobotNode::RobotSpeed speed_value = m_robot_node->getSpeedNormalized();       // -1.0 = rückwärts, 1.0 = vorwärts bei linear.x
            double rotation_value = m_robot_node->getRotationNormalized(); // -1.0 = links, 1.0 = rechts

            if (speed_value.x < 0.0) {
                rotation_value = -rotation_value; // Lenkradspiegelung rückwärts
            }

            // Winkel berechnen (0° = rechts, 90° = oben, 180° = links, 270° = unten)
            // Ziel: 1.0 = 180°,  0.0 = 270°, -1.0 = 360°
            double rotation = 270.0 - rotation_value * 90.0;

            // Startpunkt für Pfeil – z. B. untere Bildmitte
            cv::Point start((frame.cols - 20) / 2, frame.rows - 100);

            // Rotation in Bogenmaß umwandeln
            double angle_rad = rotation * CV_PI / 180.0;

            // Richtungseinheitsvektor (Y-Achse im Bild nach oben = negativ)
            cv::Point2f dir(-std::cos(angle_rad), std::sin(angle_rad));

            // Maximale Pfeillänge in Pixeln
            double max_length = 120.0;

            // Geschwindigkeit in Pfeillänge skalieren (auch negativ möglich!)
            double length = speed_value.x * max_length;

            // Endpunkt berechnen
            cv::Point end = start + cv::Point(dir.x * length, dir.y * length);

            // Pfeilfarbe je nach Richtung: Blau für vorwärts, Rot für rückwärts
            cv::Scalar arrow_color = (speed_value.x >= 0.0) ? cv::Scalar(255, 0, 0)   // Blau (BGR)
                                                        : cv::Scalar(0, 0, 255);  // Rot

            // Optional: minimale sichtbare Länge, damit bei kleinen Werten noch etwas angezeigt wird
            if (std::abs(speed_value.x) > 0.01) {
                int thickness = 6;
                // Nur anzeigen wenn vectorMode
                if (m_vectorMode)
                    cv::arrowedLine(frame, start, end, arrow_color, thickness);
            }
            // Ende Vektorpfeil

            // === Fahrspurhilfslinie für parkingMode ===
            if (m_parkingMode) {
                double wheel_base = 50.0; 
                double max_steering_angle = M_PI / 4; // 45 Grad
                double steering_angle = rotation_value * max_steering_angle;

                // Kameraparameter (vereinfacht, anpassbar)
                double fx = 200.0;
                double fy = 200.0;
                double cx = frame.cols / 2.0;
                double cy = frame.rows / 2.0;

                // Kamera ist auf 30 cm Höhe
                double camera_height = 50.0; // in cm

                // Linienlänge in Z-Richtung (Tiefe) | abhängig von Speed
                double max_z = std::min(150.0, 60.0 + std::abs(speed_value.x) * 100); // in cm
                int num_points = 30;
                double offset = 25.0; // Abstand paralleler Linien (in cm)

                std::vector<cv::Point> left_pts;
                std::vector<cv::Point> right_pts;

                std::vector<cv::Point2f> base;

                for (int i = 0; i < num_points; ++i) {
                    double z = (i / (double)num_points) * max_z;
                    double x = 0.0;

                    if (std::abs(steering_angle) > 1e-3) {
                        double R = wheel_base / std::tan(steering_angle);
                        double theta = z / R;

                        x = R * std::sin(theta);
                        z = R * (1 - std::cos(theta));
                    } else {
                        z = 0.0;
                        x = (i / (double)num_points) * max_z;
                    }

                    base.emplace_back(x,z);
                }
                
                double y_cam = camera_height;

                for (auto &b : base) {
                    double len = hypot(b.x, b.y);
                    double perp_z =  b.y/len;
                    double perp_x = -b.x/len;

                    cv::Point2f L(b.y + perp_x*offset,
                                b.x + perp_z*offset);
                    cv::Point2f R(b.y - perp_x*offset,
                                b.x - perp_z*offset);

                    if (L.y>0) left_pts.emplace_back(projectToImage(L.x,y_cam,L.y,fx,fy,cx,cy));
                    if (R.y>0) right_pts.emplace_back(projectToImage(R.x,y_cam,R.y,fx,fy,cx,cy));
                }
                
                // Linien zeichnen
                cv::polylines(frame, left_pts, false, cv::Scalar(0, 255, 0), 3);
                cv::polylines(frame, right_pts, false, cv::Scalar(0, 255, 0), 3);
            }
            // === Ende Fahrspurhilfslinie ===



            // Kamera-Bild anzeigen
            //
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);  // Qt verwendet RGB

            QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
            QPixmap pixmap = QPixmap::fromImage(qimg);
            
            // Skalieren der Pixmap auf die Größe des Labels (auch wenn das Bild pixelig wird)
            QPixmap scaledPixmap = pixmap.scaled(ui->cam_label->size(), Qt::KeepAspectRatio, Qt::FastTransformation);

            // Wichtig: Alles, was GUI betrifft, muss im Qt-Thread passieren
            QMetaObject::invokeMethod(this, [this, scaledPixmap]() {
                ui->cam_label->setPixmap(scaledPixmap);
            }, Qt::QueuedConnection);
        }
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(m_robot_node->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

// Optionen-Fenster für Steuerungen
void MainWindow::on_mode_list_itemSelectionChanged()
{
    // Hole alle aktuell ausgewählten Items
    QList<QListWidgetItem*> selectedItems = ui->mode_list->selectedItems();

    // Flags, ob die jeweiligen Steuerelemente angezeigt werden sollen
    bool showWheel = false;
    bool defaultWheel = false;
    bool raceWheel = false;
    bool showJoystick = false;
    bool showButtons = false;
    bool showSliders = false;

    // Prüfe die ausgewählten Items
    for (QListWidgetItem *item : selectedItems) {
        QString text = item->text();
        if (text == "Default Lenkrad") {
            showWheel = true;
            defaultWheel = true;
        } else if (text == "Rennauto Lenkrad") {
            showWheel = true;
            raceWheel = true;
        } else if (text == "Joystick") {
            showJoystick = true;
        } else if (text == "Buttons") {
            showButtons = true;
        } else if (text == "Slider") {
            showSliders = true;
        }
    }

    // Steuerungswidgets sichtbar oder unsichtbar machen
    //ui->wheels->setVisible(showWheel);
    //ui->speed_slider_wheels->setVisible(showWheel);
    ui->WheelsLayout->setVisible(showWheel);
    ui->wheels->setVisible(defaultWheel);
    ui->wheels_2->setVisible(raceWheel);
    ui->JoystickLayout->setVisible(showJoystick);
    ui->sliders->setVisible(showSliders);
    ui->ButtonsLayoutHorizontal->setVisible(showButtons);
}

// Optionen Fenster für Anzeigenp
void MainWindow::on_mode_list_view_itemSelectionChanged() 
{
    // Hole alle aktuell ausgewählten Items
    QList<QListWidgetItem*> selectedItems = ui->mode_list_view->selectedItems();

    // Flags, ob die jeweiligen Steuerelemente angezeigt werden sollen
    bool showCam = false;
    bool showMap = false;


    // Prüfe die ausgewählten Items
    for (QListWidgetItem *item : selectedItems) {
        QString text = item->text();
        if (text == "Camera") {
            showCam = true;
        } else if (text == "Map") {
            showMap = true;
        }
    }

    // Anzeigewidgets sichtbar oder unsichtbar machen
    ui->cam_widget->setVisible(showCam);
    ui->ObstacleMapLayout->setVisible(showMap);
}

// Optionen Fenster der obstacle map, zum Auswählen der Funktionen
void MainWindow::on_obstacle_map_list_itemSelectionChanged()
{
    // Hole alle aktuell ausgewählten Items
    QList<QListWidgetItem*> selectedItems = ui->obstacle_map_list->selectedItems();

    // Flags, ob die jeweiligen Steuerelemente angezeigt werden sollen
    bool drawPathMode = false;
    bool beamMode = false;
    bool followMode = false;
    bool ghostMode = false;

    // Prüfe die ausgewählten Items
    for (QListWidgetItem *item : selectedItems) {
        QString text = item->text();
        if (text == "Draw path") {
            drawPathMode = true;
        } else if (text == "Beam") {
            beamMode = true;
        } else if (text == "Follow Finger") {
            followMode = true;
        } else if (text == "Ghost Robot") {
            ghostMode = true;
        }
    }

    // Anzeigewidgets sichtbar oder unsichtbar machen
    ui->obstacle_map_widget->setDrawPathMode(drawPathMode);
    ui->obstacle_map_widget->setBeamMode(beamMode);
    ui->obstacle_map_widget->setFollowMode(followMode);
    ui->obstacle_map_widget->setGhostMode(ghostMode);

    // Parmetrisierungen
    ui->curve_gain_slider->setVisible(ghostMode);
    ui->curve_gain_label->setVisible(ghostMode);
    ui->ghost_duration_slider->setVisible(ghostMode);
    ui->ghost_duration_label->setVisible(ghostMode);
    ui->beam_color_button->setVisible(beamMode);
    ui->laser_number_label->setVisible(beamMode);
    ui->laser_number_slider->setVisible(beamMode);
}

// Optionen Fenster des Kamerabildes, zum Auswählen der Funktionen
void MainWindow::on_cam_list_itemSelectionChanged()
{
    // Hole alle aktuell ausgewählten Items
    QList<QListWidgetItem*> selectedItems = ui->cam_list->selectedItems();

    // Flags, ob die jeweiligen Assitenzsysteme angezeigt werden sollen
    m_vectorMode = false;
    m_parkingMode = false;

    // Prüfe die ausgewählten Items
    for (QListWidgetItem *item : selectedItems) {
        QString text = item->text();
        if (text == "Vektorpfeil") {
            m_vectorMode = true;
        } else if (text == "Parkmodus") {
            m_parkingMode = true;
        } else if (text == "Tap-Control") {
            m_tapControlMode = true;
        }
    }
}

// Optionen Button
void MainWindow::on_modes_button_clicked() {
    // Mode-List aktivieren/deaktivieren
    ui->AllOptionsLayout->setVisible(!ui->AllOptionsLayout->isVisible());
}

// Speed Buttons
//

void MainWindow::on_fast_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 1.0; m_robot_node->publish_velocity({1.0,0.0}, m_robot_node->getRotationNormalized());}
void MainWindow::on_slow_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 0.5; m_robot_node->publish_velocity({0.5,0.0}, m_robot_node->getRotationNormalized());}
void MainWindow::on_stop_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 0.0; m_robot_node->publish_velocity({0.0,0.0}, m_robot_node->getRotationNormalized());}
void MainWindow::on_back_slow_button_clicked() { qDebug() << "Button Geschwindigkeit: " << -0.5; m_robot_node->publish_velocity({-0.5,0.0}, m_robot_node->getRotationNormalized());}
void MainWindow::on_back_fast_button_clicked() { qDebug() << "Button Geschwindigkeit: " << -1.0; m_robot_node->publish_velocity({-1.0,0.0}, m_robot_node->getRotationNormalized());}

// Reset Rotation Button
void MainWindow::on_reset_rotation_button_clicked() {
    m_robot_node->publish_velocity(m_robot_node->getSpeedNormalized(), 0.0);
}

// Parameter Slider
//
void MainWindow::on_curve_gain_slider_valueChanged(int value) {
    ui->obstacle_map_widget->setCurveGain(static_cast<double>(value)/100);
    ui->curve_gain_label->setText(QString("Curve Gain: %1").arg(value));
}

void MainWindow::on_ghost_duration_slider_valueChanged(int value) {
    ui->obstacle_map_widget->setGhostDuration(static_cast<double>(value));
    ui->ghost_duration_label->setText(QString("Ghost duration: %1").arg(value));
}

void MainWindow::on_laser_number_slider_valueChanged(int value) {
    ui->obstacle_map_widget->setLaserNumber(value);
    ui->laser_number_label->setText(QString("Laser number: %1").arg(value));
}

// Beam Color Button
void MainWindow::on_beam_color_button_clicked()
{
    QColor color = QColorDialog::getColor(Qt::red, this, "Farbe wählen");

    if (color.isValid()) {
        // Farbe erfolgreich gewählt
        qDebug() << "Gewählte Farbe:" << color;

        // Button-Hintergrund ändern
        QString qss = QString("background-color: %1").arg(color.name());
        ui->beam_color_button->setStyleSheet(qss);

        // An Obstacle Map senden
        ui->obstacle_map_widget->setLaserColor(color); 
    }
}

