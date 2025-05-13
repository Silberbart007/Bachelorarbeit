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


# Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOUIC ON)

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
  src/touch_slider.cpp
)

set(HEADERS
  include/test_qt_pkg/mainwindow.h
  include/test_qt_pkg/joystick.h
  include/test_qt_pkg/wheel.h
  include/test_qt_pkg/movable_widget.h
  include/test_qt_pkg/cam.h
  include/test_qt_pkg/obstacle_map_widget.h
  include/test_qt_pkg/touch_test.h
  include/test_qt_pkg/touch_slider.h
)

set(UIS
    ui/mainwindow.ui
    )

# Executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${UIS})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME} 
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
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
