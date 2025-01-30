cmake_minimum_required(VERSION 3.8)
project(imu_fusion_ros)

# For C++14 or later
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(imu_fusion_ros_interface REQUIRED)

# If you placed the C files from Fusion, do something like:
set(FUSION_SRC
  include/Fusion/Fusion/FusionAhrs.c
  # include/Fusion/Fusion/FusionCalibration.c
  include/Fusion/Fusion/FusionCompass.c
  # include/Fusion/Fusion/FusionMath.c
  include/Fusion/Fusion/FusionOffset.c
)

include_directories(
  include  # Contains Fusion library headers
)

add_executable(imu_fusion_node src/imu_fusion_node.cpp
  ${FUSION_SRC}   # Uncomment if you have the .c files
)

ament_target_dependencies(imu_fusion_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  imu_fusion_ros_interface
)

install(TARGETS
  imu_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
