cmake_minimum_required(VERSION 3.8)
project(my_first_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

set(CMAKE_AUTOMOC ON)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(my_noed src/my_noed.cpp)

add_executable(test_UI 
src/test_UI.cpp
src/plus_minus.cpp)

# Nur ROS-Dependencies hier!
ament_target_dependencies(my_noed
  rclcpp
  std_msgs
  rclcpp_components
)

ament_target_dependencies(test_UI
  rclcpp
  std_msgs
  rclcpp_components
)

# Qt separat verlinken
target_link_libraries(my_noed
  Qt5::Core
  Qt5::Widgets
)

target_link_libraries(test_UI
  Qt5::Core
  Qt5::Widgets
)

target_include_directories(my_noed PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(my_noed PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS my_noed
  DESTINATION lib/${PROJECT_NAME})

  target_include_directories(test_UI PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_UI PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS test_UI
  DESTINATION lib/${PROJECT_NAME})

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
