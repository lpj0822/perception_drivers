cmake_minimum_required(VERSION 2.8.3)
project(protobuf_c)

option(HOLO_EXPORT_BUILDTREE_TARGET "" ON)
SET(CMAKE_INSTALL_PREFIX "/opt/holo/3rdparty")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall -Wextra -Werror")

add_library(protobuf_c STATIC src/pb_common.c src/pb_encode.c src/pb_decode.c)
target_include_directories(protobuf_c PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

if(HOLO_EXPORT_BUILDTREE_TARGET)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/protobuf_c-config.cmake.in ${PROJECT_BINARY_DIR}/cmake/protobuf_c-config.cmake @ONLY)
    export(TARGETS protobuf_c FILE cmake/protobuf_c-targets.cmake)
    export(PACKAGE protobuf_c)
endif()

FILE(GLOB HEADER_FILES "include/*.h")
INSTALL(FILES ${PROJECT_BINARY_DIR}/cmake/protobuf_c-config.cmake DESTINATION lib/cmake/protobuf_c)
INSTALL(FILES ${HEADER_FILES} DESTINATION include/3rdparty/protobuf_c)
INSTALL(TARGETS protobuf_c EXPORT protobuf_c-targets DESTINATION lib)
INSTALL(EXPORT protobuf_c-targets DESTINATION lib/cmake/protobuf_c)
