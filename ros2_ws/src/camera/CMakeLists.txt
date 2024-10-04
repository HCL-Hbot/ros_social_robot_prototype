cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_lld src/camera_lld.cpp)
ament_target_dependencies(camera_lld rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(camera_hld src/camera_hld.cpp)
ament_target_dependencies(camera_hld rclcpp sensor_msgs cv_bridge OpenCV )

target_include_directories(camera_lld PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(camera_hld PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(camera_lld PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(camera_hld PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  camera_lld
  camera_hld
  DESTINATION lib/${PROJECT_NAME}
  )


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
