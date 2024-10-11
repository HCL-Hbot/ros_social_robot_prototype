cmake_minimum_required(VERSION 3.8)
project(camera_hld)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(msg_files
  "msg/FaceInfo.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(camera_hld_node src/main.cpp src/camera_hld.cpp)

target_include_directories(camera_hld_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(camera_hld_node rclcpp sensor_msgs cv_bridge OpenCV )
target_compile_features(camera_hld_node PUBLIC cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  camera_hld_node
  DESTINATION lib/${PROJECT_NAME}
  )

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(camera_hld_node "${cpp_typesupport_target}")

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)  # Voeg gtest toe

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Voeg de test executable toe
  ament_add_gtest(test_camera_hld test/test_camera_hld.cpp)

  target_include_directories(test_camera_hld PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
  #target_link_libraries(test_camera_hld name_of_local_library)

endif()

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_cmake_gtest REQUIRED)  # Voeg gtest toe

#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   # Voeg de test executable toe
#   ament_add_gtest(test_camera_hld test/test_camera_hld.cpp)

#   # Controleer of de test executable correct is toegevoegd en link de benodigde libraries
#   if(TARGET test_camera_hld)
#     target_link_libraries(test_camera_hld camera_hld_node)
#     ament_target_dependencies(test_camera_hld rclcpp sensor_msgs cv_bridge OpenCV)
#   endif()
# endif()

ament_package()
