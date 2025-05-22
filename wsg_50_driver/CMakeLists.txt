cmake_minimum_required(VERSION 3.8)
project(wsg_50_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(wsg_50_common REQUIRED)

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

set(DRIVER_SOURCES
  src/checksum.cpp
  src/cmd.c
  src/common.cpp
  src/functions.cpp
  src/interface.cpp
  src/msg.c
  src/tcp.c
  src/udp.c
  src/serial.c
)


# let the compiler search for headers in the include folder
include_directories(include)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
 
# define a library target called wsg_50_driver
add_library(wsg_50_driver ${DRIVER_SOURCES}) 
 
# this line to exports the library
ament_export_targets(wsg_50_driver HAS_LIBRARY_TARGET)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/wsg_50_driver
  DESTINATION include
)
 
install(
  TARGETS wsg_50_driver
  EXPORT wsg_50_driver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
# add the executable target
add_executable(wsg_50_driver_node src/main.cpp ${DRIVER_SOURCES})
ament_target_dependencies(wsg_50_driver_node 
  rclcpp 
  rclcpp_action
  std_msgs 
  std_srvs 
  sensor_msgs 
  wsg_50_common)

install(TARGETS
  wsg_50_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
