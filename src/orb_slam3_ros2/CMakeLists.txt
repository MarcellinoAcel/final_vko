cmake_minimum_required(VERSION 3.10)
project(orb_slam3_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

# ORB-SLAM3 PATH (pastikan ini benar)
set(ORB_SLAM3_ROOT_DIR "$ENV{HOME}/ORB_SLAM3")

set(ORB_SLAM3_INCLUDE_DIRS
    ${ORB_SLAM3_ROOT_DIR}/include
    ${ORB_SLAM3_ROOT_DIR}/include/CameraModels  
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/DBoW2
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

set(ORB_SLAM3_LIBS
    ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib/libg2o.so
)

add_executable(orb_slam3_node src/orb_slam3_node.cpp)

target_include_directories(orb_slam3_node PUBLIC
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${ORB_SLAM3_ROOT_DIR} 
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}

  ${ORB_SLAM3_INCLUDE_DIRS}
)

target_link_libraries(orb_slam3_node
  ${ORB_SLAM3_LIBS}
  ${ORB_SLAM3_LIBRARIES}
  ${Pangolin_LIBRARIES}
  pthread
)

ament_target_dependencies(
  orb_slam3_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
)

install(TARGETS orb_slam3_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
