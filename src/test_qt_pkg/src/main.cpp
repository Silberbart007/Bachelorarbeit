#include <QApplication>
#include "../include/test_qt_pkg/cam.h"
#include "../include/test_qt_pkg/mainwindow.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Eigenschaften setzen: Skalierung
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication app(argc, argv);

    MainWindow window;
    // Touch events aktivieren
    //window.setAttribute(Qt::WA_AcceptTouchEvents);
    window.show();

     // ROS-Thread starten
    std::thread qt_thread([&]() {
        rclcpp::spin(window.getRobotNode());  // Hier wird der die Robot Node in einem seperaten Thread aufgerufen
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
