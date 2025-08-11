/**
 * @file mainwindow.h
 * @brief Main window of the application handling UI and robot control.
 *
 * This class inherits from QMainWindow and manages the main user interface,
 * including interaction with the robot via ROS2, joystick input, image display,
 * and related control components.
 *
 * @author Max Vtulkin
 * @date 2025
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// Qt includes
#include <QColorDialog>
#include <QDebug>
#include <QImage>
#include <QMainWindow>
#include <QMutex>
#include <QPixmap>
#include <QSlider>
#include <QTimer>
#include <QTouchEvent>

// ROS2 includes
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

// OpenCV includes
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

// C++ includes
#include <cmath>
#include <functional>

// Own project includes
#include "control_widget_wrapper.h"
#include "ego_widget.h"
#include "joystick.h"
#include "laser_map_widget.h"
#include "nav2client.h"
#include "robot_node.h"
#include "wheel.h"

// Forward declarations
class Nav2Client;

namespace Ui {
class MainWindow;
}

/**
 * @class MainWindow
 * @brief Main application window managing the robot control interface.
 *
 * Handles the UI elements, user input, ROS2 communication, and coordination
 * of components such as joystick and camera display.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    /**
     * @brief Constructs the main window.
     *
     * Initializes the MainWindow with an optional parent widget.
     *
     * @param parent Pointer to the parent QWidget, default is nullptr.
     */
    explicit MainWindow(QWidget* parent = nullptr);

    /**
     * @brief Destructor.
     *
     * Cleans up any allocated resources.
     */
    ~MainWindow();

    // ===== Node Getters =====

    /**
     * @brief Get the shared pointer to the RobotNode.
     *
     * Provides access to the ROS2 node used for robot communication.
     *
     * @return Shared pointer to the RobotNode.
     */
    rclcpp::Node::SharedPtr getRobotNode() const {
        return m_robot_node;
    }

    /**
     * @brief Get the shared pointer to the Nav2Client node as a generic rclcpp::Node.
     *
     * Allows access to the navigation node casted to the base rclcpp::Node.
     *
     * @return Shared pointer to the Nav2Client as rclcpp::Node.
     */
    rclcpp::Node::SharedPtr getNav2Node() const {
        return std::static_pointer_cast<rclcpp::Node>(m_nav2_node);
    }

    // ===== Public Static variables =====

    /// @brief Toggle Dynamic Lock of all active sliders
    static bool s_dynLock;

    /// @brie Toggle Static Lock of all active sliders
    static bool s_statLock;

  protected:
    /**
     * @brief Filters events sent to this object or its children.
     *
     * Used to intercept and handle specific events like mouse clicks, gestures, or touches.
     * This method overrides QWidget::eventFilter.
     *
     * @param obj Pointer to the object that sent the event.
     * @param event Pointer to the event to be processed.
     * @return true if the event is handled and should not propagate further; false otherwise.
     */
    bool eventFilter(QObject* obj, QEvent* event) override;

  private slots:
    // ===== Options List =====

    /**
     * @brief Slot triggered when the selection changes in the mode list.
     */
    void on_mode_list_2_itemSelectionChanged();

    /**
     * @brief Slot triggered when the selection changes in the mode list view.
     */
    void on_mode_list_view_itemSelectionChanged();

    /**
     * @brief Slot triggered when the selection changes in the lock options list
     */
    void on_lock_options_list_itemSelectionChanged();

    /**
     * @brief Slot triggered when the modes button is clicked.
     */
    void on_modes_button_clicked();

    // ===== Speed Buttons =====

    /**
     * @brief Slot triggered when the fast button is clicked.
     */
    void on_fast_button_pressed();
    void on_fast_button_released();

    /**
     * @brief Slot triggered when the slow button is clicked.
     */
    void on_slow_button_pressed();
    void on_slow_button_released();

    /**
     * @brief Slot triggered when the stop button is clicked.
     */
    void on_stop_button_clicked();

    /**
     * @brief Slot triggered when the red stop button is clicked.
     */
    void on_stop_full_button_2_clicked();

    /**
     * @brief Slot triggered when the back slow button is clicked.
     */
    void on_back_slow_button_pressed();
    void on_back_slow_button_released();

    /**
     * @brief Slot triggered when the back fast button is clicked.
     */
    void on_back_fast_button_pressed();
    void on_back_fast_button_released();

    /**
     * @brief Slot triggered when the clockwise slow button is clicked.
     */
    void on_clockwise_slow_button_pressed();
    void on_clockwise_slow_button_released();

    /**
     * @brief Slot triggered when the clockwise fast button is clicked.
     */
    void on_clockwise_fast_button_pressed();
    void on_clockwise_fast_button_released();

    /**
     * @brief Slot triggered when the anticlockwise slow button is clicked.
     */
    void on_anticlockwise_slow_button_pressed();
    void on_anticlockwise_slow_button_released();

    /**
     * @brief Slot triggered when the anticlockwise fast button is clicked.
     */
    void on_anticlockwise_fast_button_pressed();
    void on_anticlockwise_fast_button_released();

    // ===== Reset Rotation Button =====

    /**
     * @brief Slot triggered when the reset rotation button is clicked.
     */
    void on_reset_rotation_button_clicked();

    /**
     * @brief Slot triggered when the reset rotation button 2 (sliders) is clicked.
     */
    void on_reset_rotation_button_2_clicked();

    /**
     * @brief Slot triggered when the start timer button is clicked.
     */
    void on_start_timer_button_clicked();

    /**
     * @brief Slot triggered when the stop timer button is clicked.
     */
    void on_stop_timer_button_clicked();

    // ===== Obstacle Map List =====

    /**
     * @brief Slot triggered when the selection changes in the obstacle map list.
     */
    void on_obstacle_map_list_itemSelectionChanged();

    // ===== Camera Modes List =====

    /**
     * @brief Slot triggered when the selection changes in the camera mode list.
     */
    void on_cam_list_itemSelectionChanged();

    // ===== Parameter Sliders =====

    /**
     * @brief Slot triggered when the curve gain slider value changes.
     * @param value New slider value.
     */
    void on_curve_gain_slider_valueChanged(int value);

    /**
     * @brief Slot triggered when the ghost duration slider value changes.
     * @param value New slider value.
     */
    void on_ghost_duration_slider_valueChanged(int value);

    /**
     * @brief Slot triggered when the laser number slider value changes.
     * @param value New slider value.
     */
    void on_laser_number_slider_valueChanged(int value);

    /**
     * @brief Slot triggered when the zoom factor slider value changes.
     * @param value New slider value.
     */
    void on_zoom_factor_slider_valueChanged(int value);

    /**
     * @brief Slot triggered when the trail lifetime slider value changes.
     * @param value New slider value.
     */
    void on_trail_lifetime_slider_valueChanged(int value);

    // ===== Color Change Buttons =====

    /**
     * @brief Slot triggered when the beam color button is clicked.
     */
    void on_beam_color_button_clicked();

    /**
     * @brief Slot triggered when the trail color button is clicked.
     */
    void on_trail_color_button_clicked();

    /**
     * @brief Slot triggered when the ghost color button is clicked.
     */
    void on_ghost_color_button_clicked();

    /**
     * @brief Slot triggered when follow check box on map is clicked
     */
    void on_follow_checkBox_stateChanged(int state);

    // ===== Projection Helper Function =====

    /**
     * @brief Projects a 3D point (x, y, z) onto a 2D image plane using camera intrinsics.
     *
     * @param x X coordinate in 3D space.
     * @param y Y coordinate in 3D space.
     * @param z Z coordinate in 3D space.
     * @param fx Focal length in x direction.
     * @param fy Focal length in y direction.
     * @param cx Principal point x coordinate.
     * @param cy Principal point y coordinate.
     * @return cv::Point Projected 2D point on the image.
     */
    cv::Point projectToImage(double x, double y, double z, double fx, double fy, double cx,
                             double cy);

  private:
    /// Pointer to the UI elements generated by Qt Designer
    Ui::MainWindow* m_ui;

    /// Shared pointer to the RobotNode for robot communication
    std::shared_ptr<RobotNode> m_robot_node;

    /// Shared pointer to the Nav2Client for navigation commands
    std::shared_ptr<Nav2Client> m_nav2_node;

    // For sending velocity while holding widget
    QTimer* m_velocityTimer;

    // Current speeds of buttons
    double m_button_linear_x = 0.0;
    double m_button_linear_y = 0.0;
    double m_button_angular_z = 0.0;

    // ===== Camera Modes =====

    /// True if parking mode is active
    bool m_parkingMode;

    /// True if vector mode is active
    bool m_vectorMode;

    /// True if tap control mode is active
    bool m_tapControlMode;

    /// Logging file for laser distances
    QFile m_laser_logFile;

    /// Logging stream for laser distances
    QTextStream m_laser_logStream;

    /// Timer vor logging
    QTimer* m_timer;

    /// How much time passed since timer
    int m_timer_elapsedTenthsSecond;

    /// Struct for capturing current interaction with GUI (for logging)
    struct Interaction {
        QString widgetName;
        QPointF position;
    };

    /// Collection of current interactions with GUI (for logging)
    QVector<Interaction> m_recentInteractions;

    /// for avoiding capturing multiple logging interactions at the same time
    QMutex m_interactionMutex;

    /// Initialize Logging files
    void initLogging();

    /// Start timer
    void startTimer();

    /// Stop timer
    void stopTimer();

    /// update timer label
    void updateTimerLabel();

    /// Handles all standard stop buttons
    void handleStopButton();

    /// All logging functionality
    void logEvent();

    // For timer that sends velocity every few ms
    void sendCurrentVelocity();

    // ===== Callbacks =====

    /**
     * @brief Callback function to handle incoming image messages.
     * @param msg Shared pointer to the incoming sensor_msgs::msg::Image message.
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif // MAINWINDOW_H
