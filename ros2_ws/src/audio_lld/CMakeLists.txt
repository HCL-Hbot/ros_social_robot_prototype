cmake_minimum_required(VERSION 3.8)
project(audio_lld)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)

include_directories(
  include
  ${GSTREAMER_INCLUDE_DIRS} 
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
)

add_library(speaker src/speaker.cpp)
target_include_directories(speaker PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(speaker ${GSTREAMER_LIBRARIES})

add_executable(audio_lld_node src/audio_lld_node.cpp)
target_include_directories(audio_lld_node PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(audio_lld_node speaker ${GSTREAMER_LIBRARIES})
ament_target_dependencies(audio_lld_node rclcpp std_msgs std_srvs)

install(TARGETS
  audio_lld_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS speaker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
