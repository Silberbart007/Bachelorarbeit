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
#include <QSlider>
#include <QTouchEvent>
#include <functional>
#include <QTimer>
#include "cv_bridge/cv_bridge.hpp"
#include "joystick.h"
#include "robot_node.h"
#include "wheel.h"
#include "nav2client.h"
#include <cmath>
#include <QColorDialog>

class Nav2Client;

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
    rclcpp::Node::SharedPtr getRobotNode() const { return m_robot_node; };
    rclcpp::Node::SharedPtr getNav2Node() const { return std::static_pointer_cast<rclcpp::Node>(m_nav2_node); };

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots:

    // Optionen-Liste
    void on_mode_list_itemSelectionChanged();
    void on_mode_list_view_itemSelectionChanged();
    void on_modes_button_clicked();

    // Speed Buttons
    void on_fast_button_clicked();
    void on_slow_button_clicked();
    void on_stop_button_clicked();
    void on_back_slow_button_clicked();
    void on_back_fast_button_clicked();

    // Reset Rotation button
    void on_reset_rotation_button_clicked();

    // Obstacle Map List
    void on_obstacle_map_list_itemSelectionChanged();

    // Kameramodi-List
    void on_cam_list_itemSelectionChanged();

    // Parameter Slider
    void on_curve_gain_slider_valueChanged(int value);
    void on_ghost_duration_slider_valueChanged(int value);
    void on_laser_number_slider_valueChanged(int value);
    void on_trail_lifetime_slider_valueChanged(int value);

    // Color change für Laser
    void on_beam_color_button_clicked();

    // Color change für Trail
    void on_trail_color_button_clicked();

    // Color change für Ghost
    void on_ghost_color_button_clicked();

    // Hilfsfunktion für Projektion
    cv::Point projectToImage(double x, double y, double z,
                         double fx, double fy,
                         double cx, double cy);

private:
    Ui::MainWindow *m_ui;

    std::shared_ptr<RobotNode> m_robot_node;
    std::shared_ptr<Nav2Client> m_nav2_node;

    // Kameramodi
    bool m_parkingMode;
    bool m_vectorMode;
    bool m_tapControlMode;
    
    // Konkrete Callback Funktionen
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    // Hilfsfunktion zum Mappen der Position auf den Sliderwert
    int mapToSliderValue(int pos, QSlider* slider);

    int m_counter;
};

#endif // MAINWINDOW_H
