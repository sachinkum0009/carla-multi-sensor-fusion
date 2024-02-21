cmake_minimum_required(VERSION 3.8)
project(sensor_fusion_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ros2_numpy REQUIRED)
find_package(geometry_msgs)
find_package(tf2_ros)
find_package(tf2_geometry_msgs)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_fusion_perception)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Dimension.msg"
  "msg/RoadObject.msg"
  "msg/RoadObjects.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
 )

# Find and include OpenCV
find_package(OpenCV REQUIRED)

include_directories(
  include
)

add_executable(image_publisher_node src/image_publisher.cpp)
ament_target_dependencies(image_publisher_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(image_publisher_node ${OpenCV_LIBS})

add_executable(tf2_broadcaster_node src/tf2_broadcaster.cpp)
ament_target_dependencies(tf2_broadcaster_node rclcpp geometry_msgs tf2_geometry_msgs tf2_ros)

install(TARGETS
  image_publisher_node
  tf2_broadcaster_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/opencv_publisher.py
  src/camera_and_radar_data_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch datasets src DESTINATION share/${PROJECT_NAME})



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
