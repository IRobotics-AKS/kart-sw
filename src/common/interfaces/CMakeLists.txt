cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add reviewed msg/*.msg and srv/*.srv paths here, plus their dependencies.
# An empty list is intentional: actuator and status semantics are unresolved.
set(interface_files)
if(interface_files)
  rosidl_generate_interfaces(${PROJECT_NAME} ${interface_files})
  ament_export_dependencies(rosidl_default_runtime)
endif()

install(DIRECTORY msg srv DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
