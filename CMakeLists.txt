cmake_minimum_required(VERSION 2.8.3)
project(holomatic)

option(USE_CUDA "" ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11 -Wall -Wextra")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall -Wextra -Werror")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  roslib
  cmake_modules
  pcl_conversions
  pcl_ros
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA)

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime roslib
)

catkin_package(DEPENDS Eigen)

SET(PROTOBUF_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/protobuf_c/build/libprotobuf_c.a)
FILE(GLOB HOLO_LIB ${CMAKE_CURRENT_SOURCE_DIR}/ros/libs/*.so* ${CMAKE_CURRENT_SOURCE_DIR}/ros/libs/holo/*.so ${CMAKE_CURRENT_SOURCE_DIR}/ros/libs/*.a /usr/local/cuda/lib64/*.so)
#SET(HOLO_LIB ${CMAKE_CURRENT_SOURCE_DIR}/libs/*.so ${CMAKE_CURRENT_SOURCE_DIR}/libs/holo/*.so)

#MESSAGE(FATAL_ERROR ${HOLO_LIB})

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  3rdparty/
  3rdparty/protobuf_c/include
  3rdparty/libsocket/include
  3rdparty/serial
  3rdparty/pcl
  include/holo/holo_base/include
  include/holo/holo_sensors/include
  ${catkin_INCLUDE_DIRS}
)

if(USE_CUDA)
include_directories(
  ${CUDA_INCLUDE_DIRS}
)
endif(USE_CUDA)


##################################################
## Candev Radar Driver
##################################################
add_library(holo_sensors_candev
            src/sensors/candev/candev.cpp
)
target_link_libraries(holo_sensors_candev ${HOLO_LIB})
set(HOLO_SENSORS_TARGETS ${HOLO_SENSORS_TARGETS} holo_sensors_candev)

target_include_directories(holo_sensors_candev PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:install>)

##################################################
## Conti Radar Driver
##################################################
add_library(holo_sensors_contiradar
            src/sensors/contiradar/contiradar.cpp
)
target_link_libraries(holo_sensors_contiradar ${HOLO_LIB})
set(HOLO_SENSORS_TARGETS ${HOLO_SENSORS_TARGETS} holo_sensors_contiradar)

target_include_directories(holo_sensors_contiradar PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:install>)
##################################################
## Delphi Esr Driver
##################################################
add_library(holo_sensors_delphi_esr
            src/sensors/delphi_esr/delphi_esr.cpp
)
target_link_libraries(holo_sensors_delphi_esr ${HOLO_LIB})
set(HOLO_SENSORS_TARGETS ${HOLO_SENSORS_TARGETS} holo_sensors_delphi_esr)

target_include_directories(holo_sensors_delphi_esr PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:install>)
##################################################
## Delphi Srr3 Driver
##################################################
add_library(holo_sensors_delphi_srr3
            src/sensors/delphi_srr3/delphi_srr3.cpp
)
target_link_libraries(holo_sensors_delphi_srr3 ${HOLO_LIB})
set(HOLO_SENSORS_TARGETS ${HOLO_SENSORS_TARGETS} holo_sensors_delphi_srr3)

target_include_directories(holo_sensors_delphi_srr3 PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:install>)
##################################################
## Novatel
##################################################
add_library(holo_sensors_novatel
            src/sensors/novatel/novatel.cpp
)
target_link_libraries(holo_sensors_novatel ${HOLO_LIB})
set(HOLO_SENSORS_TARGETS ${HOLO_SENSORS_TARGETS} holo_sensors_novatel)

target_include_directories(holo_sensors_novatel PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)    
##################################################
## Pandar
##################################################
add_library(holo_sensors_pandar
            src/sensors/pandar/pandar.cpp
            src/sensors/pandar/pandar_pcd_converter.cpp
)
target_link_libraries(holo_sensors_pandar ${HOLO_LIB})
set(HOLO_SENSORS_TARGETS ${HOLO_SENSORS_TARGETS} holo_sensors_pandar)

target_include_directories(holo_sensors_pandar PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
##################################################
## Velodyne
##################################################
add_library(holo_sensors_velodyne
            src/sensors/velodyne/velodyne.cpp
            src/sensors/velodyne/velodyne_pcd_converter.cpp
)
target_link_libraries(holo_sensors_velodyne ${HOLO_LIB})
set(HOLO_SENSORS_TARGETS ${HOLO_SENSORS_TARGETS} holo_sensors_velodyne)

target_include_directories(holo_sensors_velodyne PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
##################################################
## Camera
##################################################
#include_directories(
#                    /opt/holo/include/holo_base/
#                    /opt/holo/3rdparty/include/)

link_directories(/usr/local/cuda/lib64)

cuda_compile(holo_sensors_camera_gpu
                 src/sensors/camera/image_gpu.cu
)

add_library(holo_sensors_camera
            src/sensors/camera/image_sync_ipc.cpp
            src/sensors/camera/image_capture.cpp
            src/sensors/camera/image_sync.cpp
            src/sensors/camera/image_utils.cpp
            ${holo_sensors_camera_gpu}
            #src/sensors/camera/image_compress.cpp
            #src/sensors/camera/image_decompress.cpp
            src/sensors/camera/raw2bmp.c
)
target_link_libraries(holo_sensors_camera yaml-cpp glog ${HOLO_LIB} ${CUDA_LIBRARIES})# ${FFMPEG_LIBRARIES} -ljpeg -lx264) 
set(HOLO_SENSORS_TARGETS ${HOLO_SENSORS_TARGETS} holo_sensors_camera)

target_include_directories(holo_sensors_camera PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:install>)
