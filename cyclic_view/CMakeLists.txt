cmake_minimum_required(VERSION 3.10)
project(cyclic_view VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include Catch2
include(FetchContent)
FetchContent_Declare(
  catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v2.13.6
)
FetchContent_MakeAvailable(catch2)

# Add the library
add_library(cyclic_view INTERFACE)
target_include_directories(cyclic_view INTERFACE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add compile options
add_compile_options(-Wformat -Wall -Wextra -pedantic -Werror -Wconversion -Wshadow -Wunreachable-code -Wunused -Wunused-function)

# Add the test
add_executable(cyclic_viewTest test/test_cyclic_view.cpp)
target_include_directories(cyclic_viewTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cyclic_viewTest PRIVATE Catch2::Catch2)

# Add sanitizer options for testing
target_compile_options(cyclic_viewTest PRIVATE -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer)
target_link_options(cyclic_viewTest PRIVATE -fsanitize=address -fsanitize=undefined)

# Ensure ASan runtime is linked
target_link_libraries(cyclic_viewTest PRIVATE -static-libasan)

# Enable testing
enable_testing()
add_test(NAME cyclic_viewTest COMMAND cyclic_viewTest)

# Install the library
install(TARGETS cyclic_view
        EXPORT cyclic_viewTargets
        INCLUDES DESTINATION include)

# Export the targets to a file
install(EXPORT cyclic_viewTargets
        FILE cyclic_viewTargets.cmake
        NAMESPACE cyclic_view::
        DESTINATION lib/cmake/cyclic_view)

# Install the header file
install(FILES include/cyclic_view/cyclic_view.hpp
        DESTINATION include/cyclic_view)

# Ensure test files are installed
install(TARGETS cyclic_viewTest
        DESTINATION bin)

# Create and install the package configuration file
include(CMakePackageConfigHelpers)
set(cyclic_view_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/cyclic_viewConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY AnyNewerVersion
)

configure_file(cyclic_viewConfig.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/cyclic_viewConfig.cmake" @ONLY)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/cyclic_viewConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/cyclic_viewConfigVersion.cmake"
  DESTINATION lib/cmake/cyclic_view
)
