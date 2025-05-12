#include <QApplication>
#include "../include/test_qt_pkg/cam.h"
#include "../include/test_qt_pkg/mainwindow.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    MainWindow window;
    window.show();

     // ROS-Thread für die Qt-Node starten
    std::thread qt_thread([&]() {
        rclcpp::spin(window.getNode());  // Hier wird der ROS 2 Node aus dem MainWindow abgerufen
    });

    // ROS-Thread für den Kamera-Publisher starten
    std::thread camera_thread([&]() {
        start_camera_node();  // Cam publisher Node aufrufen
    });

    // Qt Event-Loop starten
    int ret = app.exec();  // Event-Loop von Qt ausführen

    qt_thread.join();  // ROS-Thread beenden
    camera_thread.join();
    rclcpp::shutdown();  // ROS 2 sauber beenden
    return ret;
}
