cmake_minimum_required(VERSION 3.8)
project(eye_display_lld)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ----------------------------------
# Interface messages
# ----------------------------------

set(msg_files
 "msg/PupilControl.msg"
 "msg/EyesDirection.msg"
 "msg/EyeLidControl.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
   ${msg_files}
   DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} rosidl_typesupport_cpp)

# ----------------------------------
# Install msg 
# ----------------------------------

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------------
# Install launch files (when installation is done)
# ----------------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------------
# Electron UI - development install
# ----------------------------------

# Ensure Node.js and npm are available
# Check at configure time
find_program(NPM_EXECUTABLE npm)
if(NOT NPM_EXECUTABLE)
  message(FATAL_ERROR "npm not found. Make sure Node.js and npm are installed.")
endif()

# Run npm install during the install phase
install(CODE "
  message(STATUS \" Running 'npm install' in: ${CMAKE_CURRENT_SOURCE_DIR}/src\")

  execute_process(
    COMMAND ${NPM_EXECUTABLE} install
    WORKING_DIRECTORY \"${CMAKE_CURRENT_SOURCE_DIR}/src\"
    RESULT_VARIABLE npm_install_result
    OUTPUT_VARIABLE npm_install_output
    ERROR_VARIABLE npm_install_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
  )

  if(npm_install_result)
    message(FATAL_ERROR \"npm install failed: \${npm_install_error}\")
  else()
    message(STATUS \"npm install succeeded: \${npm_install_output}\")
  endif()
")

ament_package()
