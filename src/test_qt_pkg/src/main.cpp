#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // ROS2 Initialisierung

    // Eigenschaften setzen: Skalierung
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    std::thread robot_node_thread([&]() {
        rclcpp::spin(window.getRobotNode());  // Spin in eigenem Thread
    });

    // Qt Event-Loop starten
    int ret = app.exec();  // Event-Loop von Qt ausführen

    rclcpp::shutdown();
    robot_node_thread.join();

    return ret;
}
