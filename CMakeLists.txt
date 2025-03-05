cmake_minimum_required(VERSION 3.5)
project(radiacode_driver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
set(REQUIREMENTS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt)

# Add a custom command to install dependencies
execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pip install -r ${REQUIREMENTS_FILE}
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Spectrum.msg"
  DEPENDENCIES builtin_interfaces
)

install(
  DIRECTORY radiacode
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/radiacode_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
