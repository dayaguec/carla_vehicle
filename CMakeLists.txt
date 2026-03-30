cmake_minimum_required(VERSION 3.8)
project(carla_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vehicle_interfaces REQUIRED)
find_package(perception_interfaces REQUIRED)
find_package(awareness_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ad_map_access REQUIRED)

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

include_directories(
  include
)

add_executable(image_transport_node src/image_transport_node.cpp)
ament_target_dependencies(image_transport_node rclcpp image_transport yaml-cpp sensor_msgs)
target_link_libraries(image_transport_node ${YAML_CPP_LIBRARIES})

add_executable(front_axis_transform_node src/front_axis_transform_node.cpp)
ament_target_dependencies(front_axis_transform_node rclcpp geometry_msgs nav_msgs tf2_ros tf2)

add_executable(ackermann_wheel_odometry_node src/ackermann_wheel_odometry_node.cpp)
ament_target_dependencies(ackermann_wheel_odometry_node rclcpp nav_msgs geometry_msgs tf2_geometry_msgs vehicle_interfaces)

add_executable(geo_converter_node src/geo_converter_node.cpp)
ament_target_dependencies(geo_converter_node rclcpp nav_msgs sensor_msgs ad_map_access)
target_link_libraries(geo_converter_node ad_map_access)

ament_python_install_package(carla_vehicle)

# Install Python executables
install(PROGRAMS
  scripts/vehicle_node.py
  scripts/carla_ackermann_node.py
  scripts/camera_pose.py
  scripts/carla_keyboard_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install
install(DIRECTORY rviz launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  image_transport_node
  ackermann_wheel_odometry_node
  geo_converter_node
  front_axis_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
