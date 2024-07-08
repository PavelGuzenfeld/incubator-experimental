cmake_minimum_required(VERSION 3.8)
project(static_image_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Imagefhd.msg"
  "msg/Image4k.msg"
  DEPENDENCIES
  builtin_interfaces
)

ament_package()
