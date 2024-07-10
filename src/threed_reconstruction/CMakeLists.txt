cmake_minimum_required(VERSION 3.8)
project(threed_reconstruction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filters segmentation)

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


add_executable(cave_recon src/cave_recon.cpp)
ament_target_dependencies(
  cave_recon 
  rclcpp 
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
)
target_link_libraries(cave_recon ${PCL_LIBRARIES})

add_executable(control_demo src/control_demo.cpp)
ament_target_dependencies(
  control_demo 
  rclcpp 
  tf2_ros
)
target_link_libraries(cave_recon ${PCL_LIBRARIES})

install(TARGETS
  cave_recon
  control_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
