cmake_minimum_required(VERSION 3.8)
project(cirs_girona_cala_viuda)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cirs_girona_cala_viuda REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Depth.msg"
  "msg/Imu.msg"
  "msg/LinkquestDvl.msg"
  "msg/RangeImageBeam.msg"
  DEPENDENCIES std_msgs
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


add_executable(dvl_process src/dvl_process.cpp)
ament_target_dependencies( dvl_process 
  rclcpp
  geometry_msgs
  cirs_girona_cala_viuda
)

add_executable(depth_process src/depth_process.cpp)
ament_target_dependencies( depth_process 
  rclcpp
  geometry_msgs
  cirs_girona_cala_viuda
)

add_executable(adis_imu_process src/adis_imu_process.cpp)
ament_target_dependencies( adis_imu_process 
  rclcpp
  sensor_msgs
)

install(TARGETS
  dvl_process
  depth_process
  adis_imu_process
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
