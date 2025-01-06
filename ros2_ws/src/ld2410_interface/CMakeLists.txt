cmake_minimum_required(VERSION 3.8)
project(ld2410_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LD2410TargetDataFrame.msg"
  "msg/LD2410TargetDataFrameArray.msg"
)

ament_package()
