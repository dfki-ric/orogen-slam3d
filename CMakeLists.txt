project(slam3d)
cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")

set(USE_DATABASES ON)
if (USE_DATABASES)
    add_definitions(-DUSE_DATABASES)
endif()

include(slam3dBase)

add_subdirectory(ruby)

