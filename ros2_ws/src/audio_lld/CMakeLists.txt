cmake_minimum_required(VERSION 3.8)
project(audio_lld)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)


find_package(rosidl_default_generators REQUIRED)

set(SERVICE_FILES
  "srv/PlayAudioFile.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SERVICE_FILES}
)

 include_directories(
   ${GSTREAMER_INCLUDE_DIRS} 
 )


add_executable(audio_file_player_node src/main.cpp src/audio_file_player_node.cpp)
target_include_directories(audio_file_player_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(audio_file_player_node ${GSTREAMER_LIBRARIES})
ament_target_dependencies(audio_file_player_node rclcpp std_srvs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(audio_file_player_node ${cpp_typesupport_target})

target_compile_features(audio_file_player_node PUBLIC cxx_std_17)  # Require C++17

install(TARGETS
  audio_file_player_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
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
