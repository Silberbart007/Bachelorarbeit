#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QImage>
#include <QPixmap>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "../include/test_qt_pkg/joystick.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // Getter für den Node
    rclcpp::Node::SharedPtr getNode() const { return node_; }

private slots:
    // Add Button
    void on_pushButton_clicked();

    // Speed/rotation slider
    void on_speed_slider_valueChanged(int value);
    void on_rotation_slider_valueChanged(int value);

    // Optionen-Liste
    void on_mode_list_itemSelectionChanged();
    void on_modes_button_clicked();

    // Speed Buttons
    void on_fast_button_clicked();
    void on_slow_button_clicked();
    void on_stop_button_clicked();
    void on_back_slow_button_clicked();
    void on_back_fast_button_clicked();


private:
    Ui::MainWindow *ui;
    JoystickWidget *joystick;

    rclcpp::Node::SharedPtr node_;

    // Test sub/pub für Adder
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

    // Subscriber für cam
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    int counter_;
};

#endif // MAINWINDOW_H
