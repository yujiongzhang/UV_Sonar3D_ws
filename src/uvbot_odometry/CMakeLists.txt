cmake_minimum_required(VERSION 3.8)
project(uvbot_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  sensor_msgs
  serial
  rclcpp
  tf2
  tf2_msgs
  tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
include/uvbot_odometry
device
crc
)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
  "device/*.*"
  "src/*.*"
  "crc/*.*"
)

add_executable(odometry_node ${SOURCES})

ament_target_dependencies(odometry_node PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# 链接jsoncpp库
# target_link_libraries(odometry_node libjsoncpp.a)

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

install(TARGETS odometry_node 
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION  lib/${PROJECT_NAME})

ament_package()
