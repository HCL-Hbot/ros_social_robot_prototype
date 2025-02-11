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
#find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


# Fetch face detector library (with FetchContent)
include(FetchContent)

FetchContent_Declare(
    face_detector
    GIT_REPOSITORY https://github.com/CLFML/Face_Detector.Cpp
    GIT_TAG main
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/Face_Detector.Cpp
)
FetchContent_MakeAvailable(face_detector)
####

# Fetch iris mesh library (with FetchContent)
FetchContent_Declare(
    iris_mesh.cpp
    GIT_REPOSITORY https://github.com/CLFML/Iris_Mesh.Cpp
    GIT_TAG main
    # Put the Iris_Mesh lib into lib/ folder
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/Iris_Mesh.Cpp
)
FetchContent_MakeAvailable(iris_mesh.cpp)
####

# set(msg_files
#   "msg/FaceInfo.msg"
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   DEPENDENCIES std_msgs geometry_msgs
# )

#ament_export_dependencies(rosidl_default_runtime)

# Voeg het pad naar de TensorFlow Lite bibliotheek toe
set(TENSORFLOW_LITE_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/Face_Detector.Cpp/external/tensorflow/lib")
# Voeg TensorFlow Lite toe
find_library(TENSORFLOW_LITE_LIBRARY tensorflowlite HINTS ${TENSORFLOW_LITE_LIB_PATH})

if(NOT TENSORFLOW_LITE_LIBRARY)
  message(FATAL_ERROR "TensorFlow Lite library not found")
endif()


add_executable(camera_hld_node src/main.cpp src/camera_hld.cpp)

target_include_directories(camera_hld_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(camera_hld_node rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(camera_hld_node ${TENSORFLOW_LITE_LIBRARY} CLFML::face_detector CLFML::iris_mesh)

target_compile_features(camera_hld_node PUBLIC cxx_std_17)  # Require C++17

install(TARGETS 
  camera_hld_node
  DESTINATION lib/${PROJECT_NAME}
  )

# Installeer de TensorFlow Lite bibliotheek
install(
  FILES ${TENSORFLOW_LITE_LIB_PATH}/libtensorflowlite.so
  DESTINATION lib
)
#install(FILES bindings/ros2/set_ld_path.sh DESTINATION lib/${PROJECT_NAME})


  
# rosidl_get_typesupport_target(cpp_typesupport_target
# ${PROJECT_NAME} rosidl_typesupport_cpp)

#target_link_libraries(camera_hld_node "${cpp_typesupport_target}" CLFML::face_detector)


#Place the shared library face decetor in the lib folder so it can be found by the camera_hld_node
# install(
#     FILES ${CMAKE_BINARY_DIR}/_deps/face_detector-build/libface_detector.so
#     DESTINATION lib
# )

# install(
#     FILES ${CMAKE_BINARY_DIR}/_deps/iris_mesh.cpp-build/libiris_mesh.so
#     DESTINATION lib
# )

# Install shared library libtensorflowlite.so
# install(
#     FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/Face_Detector.Cpp/external/tensorflow/lib/libtensorflowlite.so
#     DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED) # Contains: cppcheck, cpplint, uncristify, flake8, xmllint, PEP257, lint_cmake, copyright check
#   find_package(ament_cmake_gtest REQUIRED)  # Gtest


#   # Handmatige aanroep van cppcheck met aangepaste opties
#   #add_custom_target(run_cppcheck ALL
#   #  COMMAND cppcheck --enable=all --force --inline-suppr --std=c++17 --suppress=missingIncludeSystem -I ${CMAKE_SOURCE_DIR}/include 
#   #  ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/test
#   #  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#   #  COMMENT "Running cppcheck with C++17 (handmatig)"
#   #)

#   # Disable all linters except cppcheck (from ament_lint_auto). Will use the linters later in the project.. low priority..
#   set(ament_cmake_cpplint_FOUND TRUE)     # Skip cpplint
#   set(ament_cmake_uncrustify_FOUND TRUE)  # Skip uncrustify
#   set(ament_cmake_flake8_FOUND TRUE)      # Skip flake8 (Python linter)
#   set(ament_cmake_xmllint_FOUND TRUE)     # Skip xmllint (XML linter)
#   set(ament_cmake_pep257_FOUND TRUE)      # Skip PEP257 (Python docstring linter)
#   set(ament_cmake_lint_cmake_FOUND TRUE)  # Skip lint_cmake
#   set(ament_cmake_copyright_FOUND TRUE)   # Skip copyright check
  
#   # Leave cppcheck enabled (it will run by default)
  
#   ament_lint_auto_find_test_dependencies() # enable ament_lint_auto

#   # Voeg de test executable toe
#   ament_add_gtest(${PROJECT_NAME}_test test/test_camera_hld.cpp src/camera_hld.cpp)

#   target_include_directories(${PROJECT_NAME}_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
#   )

#   ament_target_dependencies(${PROJECT_NAME}_test rclcpp sensor_msgs cv_bridge OpenCV )

#   target_link_libraries(${PROJECT_NAME}_test CLFML::face_detector)  

#   #target_link_libraries(${PROJECT_NAME}_test "${cpp_typesupport_target}" CLFML::face_detector)  

# endif()

ament_package()
