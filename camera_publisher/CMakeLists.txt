cmake_minimum_required(VERSION 3.8)
project(camera_publisher)

# Standard 23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set policy to suppress INTERPROCEDURAL_OPTIMIZATION warnings
if(POLICY CMP0069)
  cmake_policy(SET CMP0069 NEW)
endif()

# Enable LTO globally (optional)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# Use Gold linker (optional) 
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=gold")

# Compile options
add_compile_options(-Wformat -Wall -Wextra -pedantic -Werror -Wconversion -Wshadow -Wunreachable-code -Wunused -Wunused-function)
add_compile_options(-Wno-conversion) # Disable conversion warnings due to gst headers

# Debug build
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb3")


# Enable maximum optimization
add_compile_options(-O3)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Use pkg-config to find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

include_directories(${GST_INCLUDE_DIRS} ${GST_BASE_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS} include/camera_publisher)
link_directories(${GST_LIBRARY_DIRS} ${GST_BASE_LIBRARY_DIRS} ${GST_APP_LIBRARY_DIRS})

add_executable(camera_publisher src/camera_publisher.cpp src/gst_pipeline.cpp)
ament_target_dependencies(camera_publisher rclcpp sensor_msgs)
target_link_libraries(camera_publisher ${GST_LIBRARIES} ${GST_BASE_LIBRARIES} ${GST_APP_LIBRARIES})


find_package(Curses REQUIRED)
add_executable(camera_subscriber src/camera_subscriber.cpp)
ament_target_dependencies(camera_subscriber rclcpp sensor_msgs)

target_link_libraries(camera_subscriber ${CURSES_LIBRARIES})

install(TARGETS
  camera_publisher
  camera_subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
