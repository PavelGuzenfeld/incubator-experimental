cmake_minimum_required(VERSION 3.16)
project(sync-utc-px4)

# Standard 23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable LTO globally (optional)
# set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# Use Gold linker (optional) 
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=gold")

# Debug build
set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb3")

# Ignore BOOST deprecated headers
# add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
# add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")

# Compile options
# add_compile_options(-Wformat -Wall -Wextra -pedantic -Werror -Wconversion -Wshadow -Wunreachable-code -Wunused -Wunused-function -fsanitize=address)
add_compile_options(-Wformat -Wall -Wextra -pedantic -Werror -Wconversion -Wshadow -Wunreachable-code -Wunused -Wunused-function)

# add_compile_definitions(NDEBUG)
# set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
# add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
# add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

# Add compile options for optimization
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        -O3                    # Enable maximum optimization
        -march=native          # Optimize for the host machine
        -flto                  # Enable link-time optimization
        -fuse-linker-plugin    # Use linker plugin for LTO
        -funroll-loops         # Unroll loops for optimization
        -ffast-math            # Fast math optimizations
    )
endif()

# find dependencies
find_package(MAVSDK REQUIRED)

# Add include directories
include_directories(${MAVSDK_INCLUDE_DIRS} include)

add_executable(mavsdk-timesync src/mavsdk-timesync.cpp src/mavsdk_connector.cpp)
target_link_libraries(mavsdk-timesync MAVSDK::mavsdk -lrt -lpthread)

# target_link_libraries(mission_control MAVSDK::mavsdk -fsanitize=address)
# target_link_libraries(${PROJECT_NAME} MAVSDK::mavsdk -lrt -lpthread)

add_executable(${PROJECT_NAME}-timesync src/sync-utc-px4-timesync.cpp src/mavsdk_connector.cpp)
target_link_libraries(${PROJECT_NAME}-timesync MAVSDK::mavsdk -lrt -lpthread)

install(TARGETS
    mavsdk-timesync
    ${PROJECT_NAME}-timesync
    DESTINATION lib/${PROJECT_NAME}
)
