cmake_minimum_required(VERSION 3.16)
project(skp_to_stl)

set(CMAKE_CXX_STANDARD 14)

set(BUILD_BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/windows/)

include(sketchup-sdk.cmake)

set(SRC_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/skp2stl/stl_exporter.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/stl_exporter.cc
        apps/skp2stl-simple/main.cc)

add_library(skp2stl STATIC ${SRC_FILES})
target_include_directories(
        skp2stl
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(
        skp2stl
        PUBLIC
        SketchUpAPI
)

add_subdirectory(apps/skp2stl-simple)

include(InstallRequiredSystemLibraries)
