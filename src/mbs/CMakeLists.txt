cmake_minimum_required(VERSION 3.8)
project(mbs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(oculus_interfaces REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/mbs
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



add_executable(m750d_image_to_pointcloud src/m750d_image_to_pointcloud.cpp)
ament_target_dependencies(
  m750d_image_to_pointcloud 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  oculus_interfaces
)
target_link_libraries(m750d_image_to_pointcloud ${OpenCV_LIBS} )


add_executable(m750d_ping_to_image src/m750d_ping_to_image.cpp)
ament_target_dependencies(
  m750d_ping_to_image 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  oculus_interfaces
)
target_link_libraries(m750d_ping_to_image ${OpenCV_LIBS} )

add_executable(m750d_ping_to_pointcloud src/m750d_ping_to_pointcloud.cpp)
ament_target_dependencies(
  m750d_ping_to_pointcloud 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  oculus_interfaces
  OpenCV
)

install(TARGETS
  m750d_image_to_pointcloud
  m750d_ping_to_image
  m750d_ping_to_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})



ament_package()
