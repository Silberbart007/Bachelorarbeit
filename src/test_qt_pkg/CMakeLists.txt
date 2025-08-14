cmake_minimum_required(VERSION 3.8)
project(test_qt_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(path_planning_eval REQUIRED)

# Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)  # wichtig für ui-Dateien
set(CMAKE_AUTOUIC_SEARCH_PATHS ui)

find_package(Qt5 REQUIRED COMPONENTS Widgets)


# Quellen
set(SOURCES
  src/main.cpp
  src/mainwindow.cpp
  src/joystick.cpp
  src/wheel.cpp
  src/movable_widget.cpp
  src/cam.cpp
  src/obstacle_map_widget.cpp
  src/touch_test.cpp
  src/touch_slider_vertical.cpp
  src/touch_slider_horizontal.cpp
  src/stop_button.cpp
  src/robot_node.cpp
  src/control_widget_wrapper.cpp
  src/tcp_vel_client.cpp
  src/tcp_robot_listener.cpp
  src/compressed_to_raw.cpp
  src/cmd_vel_relay.cpp
  src/ego_widget.cpp
  src/laser_map_widget.cpp
)

set(HEADERS
  include/test_qt_pkg/mainwindow.h
  include/test_qt_pkg/joystick.h
  include/test_qt_pkg/wheel.h
  include/test_qt_pkg/movable_widget.h
  include/test_qt_pkg/cam.h
  include/test_qt_pkg/obstacle_map_widget.h
  include/test_qt_pkg/touch_test.h
  include/test_qt_pkg/touch_slider_vertical.h
  include/test_qt_pkg/touch_slider_horizontal.h
  include/test_qt_pkg/robot_node.h
  include/test_qt_pkg/nav2client.h
  include/test_qt_pkg/scan_zone.h
  include/test_qt_pkg/stop_button.h
  include/test_qt_pkg/control_widget_wrapper.h
  include/test_qt_pkg/bridge_protocol.h
  include/test_qt_pkg/ego_widget.h
  include/test_qt_pkg/laser_map_widget.h
)

set(UIS
    ui/mainwindow.ui
    )


# GUI Executable (mit main.cpp)
add_executable(gui_main
  src/main.cpp
  src/robot_node.cpp
  src/nav2client.cpp
  src/mainwindow.cpp
  src/joystick.cpp
  src/wheel.cpp
  src/movable_widget.cpp
  src/obstacle_map_widget.cpp
  src/touch_test.cpp
  src/touch_slider_vertical.cpp
  src/touch_slider_horizontal.cpp
  src/stop_button.cpp
  src/control_widget_wrapper.cpp
  src/ego_widget.cpp
  src/laser_map_widget.cpp
  ui/mainwindow.ui
  ${HEADERS}
)

ament_target_dependencies(gui_main
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  nav_msgs
  nav2_amcl
  nav2_map_server
  rclcpp_action
  rosidl_default_generators
  path_planning_eval
)


#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(gui_main
  Qt5::Widgets
  ${OpenCV_LIBS}
  "${cpp_typesupport_target}"
)

# Camera Node
add_executable(camera_node src/cam.cpp)
ament_target_dependencies(camera_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Robot TCP listener Node
add_executable(tcp_robot_listener_node src/tcp_robot_listener.cpp)
ament_target_dependencies(tcp_robot_listener_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

# Tcp vel client Node
add_executable(tcp_vel_client_node src/tcp_vel_client.cpp)
ament_target_dependencies(tcp_vel_client_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# CompressedToRaw Node
add_executable(compressed_to_raw_node src/compressed_to_raw.cpp)
ament_target_dependencies(compressed_to_raw_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

# CmdVelRelay Node
add_executable(cmd_vel_relay_node src/cmd_vel_relay.cpp)
ament_target_dependencies(cmd_vel_relay_node
  rclcpp
  geometry_msgs
)

# Header files für die executables
target_include_directories(gui_main PUBLIC include/test_qt_pkg ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(camera_node PUBLIC include/test_qt_pkg)
target_include_directories(tcp_robot_listener_node PUBLIC include/test_qt_pkg)
target_include_directories(tcp_vel_client_node PUBLIC include/test_qt_pkg)
target_include_directories(compressed_to_raw_node PUBLIC include/test_qt_pkg)
target_include_directories(cmd_vel_relay_node PUBLIC include/test_qt_pkg)


# Installation
install(TARGETS
  gui_main
  camera_node
  tcp_robot_listener_node
  tcp_vel_client_node
  compressed_to_raw_node
  cmd_vel_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

# for amcl yaml
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
