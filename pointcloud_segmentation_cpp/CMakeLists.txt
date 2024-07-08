cmake_minimum_required(VERSION 3.8)
project(pointcloud_segmentation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pointcloud_segmentation_cpp REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core common features filters io segmentation surface)

include_directories(
  include
  dbscan
  dbscan/vendor
)


add_executable(pointcloud_segmentation_node src/pointcloud_segmentation_node.cpp dbscan/dbscan.cpp)
ament_target_dependencies(pointcloud_segmentation_node rclcpp std_msgs sensor_msgs PCL pointcloud_segmentation_cpp pcl_conversions)

install(TARGETS
  pointcloud_segmentation_node
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
