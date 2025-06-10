cmake_minimum_required(VERSION 3.21)
project(static_image_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Add the compiler option to force include <cstdint>
# This is a workaround for the missing uintX_t types, likely due to
# GCC 15 incompatibility with older ROS 2 Humble code/generators.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-include cstdint)
else()
  message(WARNING "Compiler is not GCC or Clang, the -include cstdint flag may not be supported. \
                    You might need a different flag for your compiler to force include cstdint if you see uintX_t errors.")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Imagefhd.msg"
  "msg/Image4k.msg"
  DEPENDENCIES
  builtin_interfaces
)

ament_package()
