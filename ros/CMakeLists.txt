cmake_minimum_required(VERSION 2.8.3)
project(holomatic)

option(USE_CUDA "" ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11 -Wall -Wextra")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall -Wextra -Werror")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  roslib
  cmake_modules
  pcl_conversions
  pcl_ros
)

find_package(Eigen REQUIRED)
#find_package(CUDA)

#find_package(OpenCV 3.3.1 REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})

add_message_files(
   FILES
   CameraSync.msg
   ObstacleConti408.msg
   ObstacleConti408List.msg
   ObstacleDelphiESR.msg
   ObstacleDelphiESRList.msg
   ObstacleDelphiSRR3.msg
   ObstacleDelphiSRR3List.msg
   VelodyneBlock.msg
   VelodynePacket.msg
   VelodyneScan.msg
   VelodyneSync.msg
   PandarBlock.msg
   PandarPacket.msg
   PandarScan.msg
   PandarSync.msg
   InsPositionVelocityAttitudeExtended.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime roslib
)

catkin_package(DEPENDS Eigen)

SET(PROTOBUF_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/protobuf_c/build/libprotobuf_c.a)
FILE(GLOB HOLO_LIB ${CMAKE_CURRENT_SOURCE_DIR}/libs/*.so* ${CMAKE_CURRENT_SOURCE_DIR}/libs/holo/*.so ${CMAKE_CURRENT_SOURCE_DIR}/libs/*.a)
#SET(HOLO_LIB ${CMAKE_CURRENT_SOURCE_DIR}/libs/*.so ${CMAKE_CURRENT_SOURCE_DIR}/libs/holo/*.so)

#MESSAGE(FATAL_ERROR ${HOLO_LIB})

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  /usr/local/cuda-8.0/include
  ../3rdparty/
  ../3rdparty/protobuf_c/include
  ../3rdparty/libsocket/include
  ../include/holo/holo_base/include
  ../include/holo/holo_sensors/include
  include/
  ${catkin_INCLUDE_DIRS}
)

if(USE_CUDA)
include_directories(
  ${CUDA_INCLUDE_DIRS}
)
endif(USE_CUDA)

add_executable(novatel_driver src/sensors/novatel/novatel_driver.cpp)
target_link_libraries(novatel_driver ${catkin_LIBRARIES} ${HOLO_LIB})
add_dependencies(novatel_driver holomatic_generate_messages_cpp)

if(USE_CUDA)
add_executable(camera_driver src/sensors/camera/camera_driver.cpp)
target_link_libraries(camera_driver ${catkin_LIBRARIES} ${HOLO_LIB})
add_dependencies(camera_driver holomatic_generate_messages_cpp)
endif(USE_CUDA)

add_executable(camera_sync_driver src/sensors/camera/camera_sync_driver.cpp)
target_link_libraries(camera_sync_driver ${catkin_LIBRARIES} ${HOLO_LIB})
add_dependencies(camera_sync_driver holomatic_generate_messages_cpp)

add_executable(contiradar_driver src/sensors/contiradar/contiradar_driver.cpp)
target_link_libraries(contiradar_driver ${catkin_LIBRARIES} ${HOLO_LIB})
add_dependencies(contiradar_driver holomatic_generate_messages_cpp)

add_executable(velodyne_driver src/sensors/velodyne/velodyne_driver.cpp)
target_link_libraries(velodyne_driver ${catkin_LIBRARIES} ${HOLO_LIB})
add_dependencies(velodyne_driver holomatic_generate_messages_cpp)

add_executable(velodyne_converter_driver src/sensors/velodyne/velodyne_converter_driver.cpp)
target_link_libraries(velodyne_converter_driver ${catkin_LIBRARIES} ${HOLO_LIB})
add_dependencies(velodyne_converter_driver holomatic_generate_messages_cpp)

add_executable(pandar_driver src/sensors/pandar/pandar_driver.cpp)
target_link_libraries(pandar_driver ${catkin_LIBRARIES} ${HOLO_LIB})
add_dependencies(pandar_driver holomatic_generate_messages_cpp)

add_executable(pandar_converter_driver src/sensors/pandar/pandar_converter_driver.cpp)
target_link_libraries(pandar_converter_driver ${catkin_LIBRARIES} ${HOLO_LIB})
add_dependencies(pandar_converter_driver holomatic_generate_messages_cpp)

#add_executable(delphi_esr_driver src/sensors/delphi_esr/delphi_esr_driver.cpp)
#target_link_libraries(delphi_esr_driver ${catkin_LIBRARIES} ${HOLO_LIB})
#add_dependencies(delphi_esr_driver holomatic_generate_messages_cpp)

#add_executable(delphi_srr3_driver src/sensors/delphi_srr3/delphi_srr3_driver.cpp)
#target_link_libraries(delphi_srr3_driver ${catkin_LIBRARIES} ${HOLO_LIB})
#add_dependencies(delphi_srr3_driver holomatic_generate_messages_cpp)
