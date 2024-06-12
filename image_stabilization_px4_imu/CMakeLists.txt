cmake_minimum_required(VERSION 3.15)
project(optical_flow)

# Find GStreamer libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER gstreamer-1.0>=1.4 REQUIRED)

# Define source file
set(SOURCE_FILE optical_flow.cpp)

# Add GStreamer include directories
include_directories(${GSTREAMER_INCLUDE_DIRS})

# Add GStreamer libraries to link against
add_executable(optical_flow ${SOURCE_FILE})
target_link_libraries(optical_flow ${GSTREAMER_LIBRARIES} gstapp-1.0)
