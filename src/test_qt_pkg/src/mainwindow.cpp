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

    // Publisher erstellen
    publisher_ = node_->create_publisher<std_msgs::msg::Int32>("counter_topic", 10);

    subscription_ = node_->create_subscription<std_msgs::msg::Int32>(
        "counter_topic",
        10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            ui->counter_label->setText(QString("Empfangen: %1").arg(msg->data));
        });

    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

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

void MainWindow::on_fast_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 1.0; }
void MainWindow::on_slow_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 0.5; }
void MainWindow::on_stop_button_clicked() { qDebug() << "Button Geschwindigkeit: " << 0.0; }
void MainWindow::on_back_slow_button_clicked() { qDebug() << "Button Geschwindigkeit: " << -0.5; }
void MainWindow::on_back_fast_button_clicked() { qDebug() << "Button Geschwindigkeit: " << -1.0; }
