cmake_minimum_required(VERSION 3.8)
project(camera_lld)

#Build this package always with c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_lld_node src/main.cpp src/camera_lld.cpp)
target_include_directories(camera_lld_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(camera_lld_node rclcpp sensor_msgs cv_bridge OpenCV)
target_compile_features(camera_lld_node PUBLIC cxx_std_17)  # Require C++17


install(TARGETS 
  camera_lld_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Disable all linters except cppcheck (from ament_lint_auto)
  set(ament_cmake_cpplint_FOUND TRUE)     # Skip cpplint
  set(ament_cmake_uncrustify_FOUND TRUE)  # Skip uncrustify
  set(ament_cmake_flake8_FOUND TRUE)      # Skip flake8 (Python linter)
  set(ament_cmake_xmllint_FOUND TRUE)     # Skip xmllint (XML linter)
  set(ament_cmake_pep257_FOUND TRUE)      # Skip PEP257 (Python docstring linter)
  set(ament_cmake_lint_cmake_FOUND TRUE)  # Skip lint_cmake
  set(ament_cmake_copyright_FOUND TRUE)   # Skip copyright check
  
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
