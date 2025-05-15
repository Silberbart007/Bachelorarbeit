#include "../include/test_qt_pkg/mainwindow.h"
#include "../ui/ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    {
    // Variablen Initialisieren
    m_counter = 0;

    // Robot Node erstellen
    m_robot_node = std::make_shared<RobotNode>();

    // Kamera-Callback registrieren
    m_robot_node->on_image_received = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->image_callback(msg);  // ruft Funktion von MainWindow auf
    };

    // Scan-Callback registrieren
    m_robot_node->on_scan_received = [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        qDebug() << "Lasercallback aus Mainwindow";
    };

    // UI "aktivieren"
    ui->setupUi(this);

    // robot Node an nötige Widgets übergeben
    ui->rotation_slider->setRobotNode(m_robot_node);
    ui->speed_slider->setRobotNode(m_robot_node);

    // Steuerungen verstecken
    ui->wheels->setVisible(false);
    ui->joysticks->setVisible(false);
    ui->sliders->setVisible(false);
    ui->buttons->setVisible(false);

    // Optionsmenü verstecken
    ui->mode_list->setVisible(false);

    // Slider Default Werte
    ui->rotation_slider->setValue(0.0);
    ui->rotation_slider->setValue(0.0);

    // Erstes Element standardmäßig auswählen
    ui->mode_list->setCurrentRow(0); // Wählt das erste Item aus

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
            double speed_value = m_robot_node->getSpeedNormalized();       // -1.0 = rückwärts, 1.0 = vorwärts
            double rotation_value = m_robot_node->getRotationNormalized(); // -1.0 = links, 1.0 = rechts

            if (speed_value < 0.0) {
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
            double length = speed_value * max_length;

            // Endpunkt berechnen
            cv::Point end = start + cv::Point(dir.x * length, dir.y * length);

            // Pfeilfarbe je nach Richtung: Blau für vorwärts, Rot für rückwärts
            cv::Scalar arrow_color = (speed_value >= 0.0) ? cv::Scalar(255, 0, 0)   // Blau (BGR)
                                                        : cv::Scalar(0, 0, 255);  // Rot

            // Optional: minimale sichtbare Länge, damit bei kleinen Werten noch etwas angezeigt wird
            if (std::abs(speed_value) > 0.01) {
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

// Optionen-Fenster
void MainWindow::on_mode_list_itemSelectionChanged()
{
    // Hole alle aktuell ausgewählten Items
    QList<QListWidgetItem*> selectedItems = ui->mode_list->selectedItems();

    // Flags, ob die jeweiligen Steuerelemente angezeigt werden sollen
    bool showWheel = false;
    bool showJoystick = false;
    bool showButtons = false;
    bool showSliders = false;

    // Prüfe die ausgewählten Items
    for (QListWidgetItem *item : selectedItems) {
        QString text = item->text();
        if (text == "Lenkrad") {
            showWheel = true;
        } else if (text == "Joystick") {
            showJoystick = true;
        } else if (text == "Buttons") {
            showButtons = true;
        } else if (text == "Slider") {
            showSliders = true;
        }
    }

    // Steuerungswidgets sichtbar oder unsichtbar machen
    ui->wheels->setVisible(showWheel);
    ui->joysticks->setVisible(showJoystick);
    ui->sliders->setVisible(showSliders);
    ui->buttons->setVisible(showButtons);
}

// Optionen Button
void MainWindow::on_modes_button_clicked() {
    // Mode-List aktivieren/deaktivieren
    ui->mode_list->setVisible(!ui->mode_list->isVisible());
}

// Speed Buttons
//

void MainWindow::on_fast_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 1.0; m_robot_node->publish_velocity(1.0, m_robot_node->getRotation());}
void MainWindow::on_slow_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 0.5; m_robot_node->publish_velocity(0.5, m_robot_node->getRotation());}
void MainWindow::on_stop_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 0.0; m_robot_node->publish_velocity(0.0, m_robot_node->getRotation());}
void MainWindow::on_back_slow_button_clicked() { qDebug() << "Button Geschwindigkeit: " << -0.5; m_robot_node->publish_velocity(-0.5, m_robot_node->getRotation());}
void MainWindow::on_back_fast_button_clicked() { qDebug() << "Button Geschwindigkeit: " << -1.0; m_robot_node->publish_velocity(-1.0, m_robot_node->getRotation());}
