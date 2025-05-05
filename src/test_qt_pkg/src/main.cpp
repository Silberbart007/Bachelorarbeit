#include <QApplication>
#include "../include/test_qt_pkg/mainwindow.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    
    MainWindow window;
    window.show();

    // ROS 2 Node in einem separaten Thread ausführen
    std::thread ros_thread([&]() {
        rclcpp::spin(window.getNode());  // Hier wird der ROS 2 Node aus dem MainWindow abgerufen
    });

    // Qt Event-Loop starten
    int ret = app.exec();  // Event-Loop von Qt ausführen

    ros_thread.join();  // ROS-Thread beenden
    rclcpp::shutdown();  // ROS 2 sauber beenden
    return ret;
}
