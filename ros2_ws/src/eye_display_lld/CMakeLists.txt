cmake_minimum_required(VERSION 3.8)
project(eye_display_lld)

# Ensure Node.js and npm are available
find_program(NPM_EXECUTABLE npm)
if(NOT NPM_EXECUTABLE)
  message(FATAL_ERROR "npm not found. Make sure Node.js and npm are installed.")
endif()

# Run npm install
execute_process(
  COMMAND ${NPM_EXECUTABLE} install
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src
  RESULT_VARIABLE npm_result
)

if(NOT npm_result EQUAL 0)
  message(FATAL_ERROR "npm install failed.")
endif()

# Standard ROS 2 settings
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
