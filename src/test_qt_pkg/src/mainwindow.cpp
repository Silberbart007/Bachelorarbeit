#include "../include/test_qt_pkg/mainwindow.h"
#include "../ui/ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    {
    // Variablen Initialisieren
    counter_ = 0;

    // ROS 2 Node erstellen
    node_ = rclcpp::Node::make_shared("qt_gui_node");

    // Test pub
    // Publisher erstellen
    publisher_ = node_->create_publisher<std_msgs::msg::Int32>("counter_topic", 10);

    // Test sub
    subscription_ = node_->create_subscription<std_msgs::msg::Int32>(
        "counter_topic",
        10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            ui->counter_label->setText(QString("%1").arg(msg->data));
        });

    // Kamera Sub
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", 10,
    std::bind(&MainWindow::image_callback, this, std::placeholders::_1));

    // UI "aktivieren"
    ui->setupUi(this);

    // Steuerungen verstecken
    ui->wheels->setVisible(false);
    ui->joysticks->setVisible(false);
    ui->sliders->setVisible(false);
    ui->buttons->setVisible(false);

    // Optionsmenü verstecken
    ui->mode_list->setVisible(false);

    // Slider Default Werte
    ui->rotation_slider->setValue(50);

    // Erstes Element standardmäßig auswählen
    ui->mode_list->setCurrentRow(0); // Wählt das erste Item aus
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

            // Vektorpfeil Beispiel mit Slider
            // Beispiel: Pfeil zeichnen basierend auf Sliderwerten
            int speed_value = ui->speed_slider->value();     // 0–100
            int rotation_value = ui->rotation_slider->value(); // 0–100

            // Werte normalisieren
            float rotation = (rotation_value-25) * 360.0 / 99.0;

            // Startpunkt z. B. in der Bildmitte
            cv::Point start((frame.cols-20) / 2, frame.rows-100);

            // Umrechnen der Rotation in Bogenmaß (0–360° → 0–2π)
            double angle_rad = rotation * CV_PI / 180.0;

            // Richtung als Einheitsvektor
            cv::Point2f dir(std::cos(angle_rad), -std::sin(angle_rad));  // Y-Achse invertieren (nach oben im Bild ist -Y)

            // Länge durch Speed skalieren
            // Logarithmische Länge berechnen
            float max_length = 200.0f;  // max. Pfeillänge in Pixel
            float speed_log = std::log10(speed_value + 1) * 25.0f; // Logarithmische Skalierung, +1 um Logarithmus von 0 zu vermeiden
            float length = std::min(speed_log, max_length);  // Maximale Länge begrenzen

            // Pfeil Zentrum bestimmen (unten am Bildschirm)
            cv::Point end = start + cv::Point(dir.x * length / 100.0 * max_length,
                                            dir.y * length / 100.0 * max_length);

            // Pfeil zeichnen (rot)
            int thickness = 6;

            cv::arrowedLine(frame, start, end, cv::Scalar(255, 0, 0), thickness);
            // Ende Vektorpfeil


            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);  // Qt verwendet RGB

            QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
            QPixmap pixmap = QPixmap::fromImage(qimg);

            // Wichtig: Alles, was GUI betrifft, muss im Qt-Thread passieren
            QMetaObject::invokeMethod(this, [this, pixmap]() {
                ui->cam_label->setPixmap(pixmap);
            }, Qt::QueuedConnection);
        }
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
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

// Add Button (zum testen)
void MainWindow::on_pushButton_clicked()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Add Button gedrueckt (dies ist eine Ros-Nachricht)");
    std_msgs::msg::Int32 msg;
    msg.data = counter_++;
    publisher_->publish(msg);
}

// Speed Slider
void MainWindow::on_speed_slider_valueChanged(int value) {
    // Als Debug Nachricht ausgeben
    qDebug() << "Speed Slider Geschwindigkeit: " << value;
}

// Rotation Slider
void MainWindow::on_rotation_slider_valueChanged(int value) {
    // Als Debug Nachricht ausgeben
    qDebug() << "Rotation Slider Geschwindigkeit: " << value;
}


// Speed Buttons
//

void MainWindow::on_fast_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 1.0; ui->speed_slider->setValue(100);}
void MainWindow::on_slow_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 0.5; ui->speed_slider->setValue(75);}
void MainWindow::on_stop_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 0.0; ui->speed_slider->setValue(50);}
void MainWindow::on_back_slow_button_clicked() { qDebug() << "Button Geschwindigkeit: " << -0.5; ui->speed_slider->setValue(25);}
void MainWindow::on_back_fast_button_clicked() { qDebug() << "Button Geschwindigkeit: " << -1.0; ui->speed_slider->setValue(0);}
