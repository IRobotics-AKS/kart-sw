cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# The production actuator/status contract remains unresolved.
set(interface_files "msg/SimActuatorCommand.msg")
if(interface_files)
  rosidl_generate_interfaces(${PROJECT_NAME} ${interface_files}
    DEPENDENCIES builtin_interfaces)
  ament_export_dependencies(rosidl_default_runtime)
endif()

install(DIRECTORY msg srv DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
