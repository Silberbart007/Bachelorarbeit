#include "../include/test_qt_pkg/mainwindow.h"
#include "../ui/ui_mainwindow.h"

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
    std_msgs::msg::Int32 msg;
    msg.data = counter_++;
    publisher_->publish(msg);
}

