cmake_minimum_required(VERSION 3.8)
project(interaction_controller)

#Build this package always with c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)  
find_package(tf2_ros REQUIRED)  
find_package(tf2_geometry_msgs REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED) #Contains message to receive the position of the detected person
find_package(camera_hld REQUIRED) # Contains message to communciate with camera_hld
find_package(eye_display_hld REQUIRED) #Contains message to communicate with eye_display_hld

set(msg_files
  "msg/PresenceDetection.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(interaction_controller_node src/main.cpp src/interaction_controller.cpp)

target_include_directories(interaction_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(interaction_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  interaction_controller_node
  "rclcpp"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "camera_hld"
  "eye_display_hld"
)

install(TARGETS interaction_controller_node
  DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(interaction_controller_node "${cpp_typesupport_target}")


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
