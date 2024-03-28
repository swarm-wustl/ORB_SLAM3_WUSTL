cmake_minimum_required(VERSION 3.8)
project(vslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

set(ORB_SLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3")
find_library(DBoW2_LIB NAMES libDBoW2.so PATHS "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib" NO_DEFAULT_PATH)
find_library(G2O_LIB NAMES libg2o.so PATHS "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib" NO_DEFAULT_PATH)
find_library(ORB_SLAM3_LIB NAMES libORB_SLAM3.so PATHS "${ORB_SLAM3_ROOT}/lib" NO_DEFAULT_PATH)

include_directories(
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/include/ThirdParty/Sophus
  ${ORB_SLAM3_ROOT}/ThirdParty/Sophus
  ${ORB_SLAM3_ROOT}/ThirdParty/DBoW2/DBoW2
  ${ORB_SLAM3_ROOT}/ThirdParty/DBoW2
  ${ORB_SLAM3_ROOT}/ThirdParty
  ${ORB_SLAM3_ROOT}/
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

add_executable(system src/system_node.cpp)
ament_target_dependencies(system rclcpp std_msgs nav_msgs sensor_msgs image_transport cv_bridge OpenCV)

add_executable(grid_mapper src/grid_mapper.cpp)
ament_target_dependencies(grid_mapper rclcpp std_msgs sensor_msgs geometry_msgs OpenCV)

add_executable(video_reader src/video_reader.cpp)
ament_target_dependencies(video_reader rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

# Custom messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapPoint.msg"
  "msg/KeyFrame.msg"
  "msg/Map.msg"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
  rosidl_default_generators
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

link_directories(
  ${ORB_SLAM3_ROOT}/Thirdparty/Pangolin/build
)

target_link_libraries(system
  ${cpp_typesupport_target} # custom messages
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${DBoW2_LIB}
  ${G2O_LIB}
  ${ORB_SLAM3_LIB}
  -lboost_system
  -lboost_serialization
  -lcrypto
  -lGL  # Link against OpenGL
  -lGLU  # Link against GLU
)

target_link_libraries(grid_mapper ${cpp_typesupport_target})
target_link_libraries(video_reader)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:~/ORB_SLAM3/Thirdparty/Pangolin/build")

install(TARGETS
  system
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  grid_mapper
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  video_reader
  DESTINATION lib/${PROJECT_NAME})

ament_package()
