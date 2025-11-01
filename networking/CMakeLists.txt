cmake_minimum_required(VERSION 3.20)
project(networking VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(fmt REQUIRED)

add_library(networking INTERFACE)

target_include_directories(networking
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include> # Path after installation
)

target_link_libraries(networking INTERFACE fmt::fmt)

# Enforce C++20 for the library and its consumers
target_compile_features(networking INTERFACE cxx_std_20)

#--------------------------------------------------------------------
# 2. INSTALL RULES
#--------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Install the header files
install(
    DIRECTORY include/networking
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Generate and install the NetworkingConfig.cmake files
# This allows `find_package(Networking)` to work for other projects
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/NetworkingConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/NetworkingConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Networking
)

# Export the target and install the export file
install(
    TARGETS networking
    EXPORT NetworkingTargets
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Networking
)

install(
    EXPORT NetworkingTargets
    FILE NetworkingTargets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Networking
)

#--------------------------------------------------------------------
# 3. TEST TARGET (ip_address_test)
#--------------------------------------------------------------------
enable_testing()

# Create the test executable
add_executable(ip_address_test test/ip_address_test.cpp)

# Link the test against our library.
# This automatically links fmt::fmt and adds the include directory.
target_link_libraries(ip_address_test PRIVATE networking)

# Add the executable as a CTest test
add_test(NAME ip_address_test COMMAND ip_address_test)