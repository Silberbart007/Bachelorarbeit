#include "mainwindow.h"
#include <QDebug>
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    {
    // Variablen Initialisieren
    m_counter = 0;

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
        //qDebug() << "Lasercallback aus Mainwindow";
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

    // Auf vollen Bildschirm (4k) aktivieren
    this->resize(3840, 2160);
}

MainWindow::~MainWindow()
{
    delete ui;
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
                cv::arrowedLine(frame, start, end, arrow_color, thickness);
            }
            // Ende Vektorpfeil

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

    // Prüfe die ausgewählten Items
    for (QListWidgetItem *item : selectedItems) {
        QString text = item->text();
        if (text == "Draw path") {
            drawPathMode = true;
        } else if (text == "Beam") {
            beamMode = true;
        } else if (text == "Follow Mode") {
            followMode = true;
        }
    }

    // Anzeigewidgets sichtbar oder unsichtbar machen
    ui->obstacle_map_widget->setDrawPathMode(drawPathMode);
    ui->obstacle_map_widget->setBeamMode(beamMode);
    ui->obstacle_map_widget->setFollowMode(followMode);
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

