cmake_minimum_required(VERSION 3.8)
project(radar_presence_hld)

#Build this package always with c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ld2410_interface REQUIRED)

set(msg_files
  "msg/PresenceDetection.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(radar_presence_hld_node src/main.cpp src/radar_presence_hld.cpp)

target_include_directories(radar_presence_hld_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(radar_presence_hld_node rclcpp ld2410_interface)
target_compile_features(radar_presence_hld_node PUBLIC cxx_std_17)  # Require C++17

install(TARGETS
radar_presence_hld_node
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(radar_presence_hld_node "${cpp_typesupport_target}")

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
