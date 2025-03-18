cmake_minimum_required(VERSION 3.8)
project(livox_organized_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED )
find_package(visualization_msgs REQUIRED)

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

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
  visualization_msgs
)
message("-----------------")
message(${PCL_FOUND})
message("-----------------")
add_executable(livox_organized_pointcloud_node src/livox_to_pointcloud2_node.cpp src/livox_to_pointcloud2.cpp)
include_directories(include/${PROJECT_NAME})  
include_directories(include   ${PCL_INCLUDE_DIRS})

# target_include_directories(livox_organized_pointcloud_node   
#   ${PCL_INCLUDE_DIRS})

target_link_libraries(livox_organized_pointcloud_node ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(livox_organized_pointcloud_node ${dependencies} pcl_conversions pcl_ros)

# ---------------- Install --------------- #
install(TARGETS livox_organized_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
