cmake_minimum_required(VERSION 3.20)
project(blockchain_ticker CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use CPM for dependency management
include(cmake/CPM.cmake)

# Add CPR
CPMAddPackage(
    NAME cpr
    GITHUB_REPOSITORY libcpr/cpr
    GIT_TAG master
)

# Add jsoncpp
CPMAddPackage(
    NAME jsoncpp
    GITHUB_REPOSITORY open-source-parsers/jsoncpp
    GIT_TAG master
)

add_executable(blockchain_ticker bitcoin.cpp)

# Link dependencies
target_link_libraries(blockchain_ticker cpr::cpr jsoncpp_lib)
