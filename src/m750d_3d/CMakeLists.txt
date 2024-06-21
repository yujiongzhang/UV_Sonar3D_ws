cmake_minimum_required(VERSION 3.8)
project(m750d_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/m750d_3d
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filters segmentation)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(m750d_3D src/m750d_3D.cpp src/image_fun.cpp)
ament_target_dependencies(
  m750d_3D 
  rclcpp 
  sensor_msgs 
  cv_bridge
  OpenCV
)
target_link_libraries(m750d_3D ${PCL_LIBRARIES})


add_executable(vertical_scan_reconstruction src/vertical_scan_reconstruction.cpp src/image_fun.cpp)
ament_target_dependencies(
  vertical_scan_reconstruction 
  rclcpp 
  sensor_msgs 
  cv_bridge
  OpenCV
  pcl_conversions
)
target_link_libraries(vertical_scan_reconstruction ${PCL_LIBRARIES})


add_executable(pc_stitch src/pc_stitch.cpp )
ament_target_dependencies(
  pc_stitch 
  rclcpp 
  sensor_msgs 
  cv_bridge
  OpenCV
)
target_link_libraries(pc_stitch ${PCL_LIBRARIES})

install(TARGETS
  m750d_3D
  vertical_scan_reconstruction
  pc_stitch
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
