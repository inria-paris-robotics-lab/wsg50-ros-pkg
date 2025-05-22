cmake_minimum_required(VERSION 3.10)
project(wsg_50_interface)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(wsg_50_driver REQUIRED)


# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# Source files
set(SOURCES
  src/wsg_ros_interface.cpp 
  src/wsg_50.cpp
)

# Library for the hardware interface
add_library(${PROJECT_NAME} SHARED ${SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  controller_interface
  ament_index_cpp
  class_loader
  wsg_50_driver
)

# Export the plugin
pluginlib_export_plugin_description_file(hardware_interface wsg_50_hardware.xml)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install shared lib
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES wsg_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  wsg_50_driver
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
