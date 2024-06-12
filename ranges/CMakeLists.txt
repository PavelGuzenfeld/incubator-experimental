cmake_minimum_required(VERSION 3.0)

project(ranges)

set(CMAKE_CXX_STANDARD 23)

find_package(Catch2 REQUIRED)

add_executable(ranges ranges.cpp)
target_link_libraries(ranges PRIVATE Catch2::Catch2WithMain)
