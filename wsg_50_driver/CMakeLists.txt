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

# WSG_50_TCP version
set(DRIVER_SOURCES
  src/checksum.cpp include/wsg_50_driver/checksum.h
  src/cmd.c include/wsg_50_driver/cmd.h
  src/common.cpp include/wsg_50_driver/common.h
  src/functions.cpp include/wsg_50_driver/functions.h
  src/interface.cpp include/wsg_50_driver/interface.h
  src/main.cpp
  src/msg.c include/wsg_50_driver/msg.h
  src/tcp.c include/wsg_50_driver/tcp.h
)
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)



add_executable(wsg_50_driver src/main.cpp ${DRIVER_SOURCES})
ament_target_dependencies(wsg_50_driver 
  rclcpp 
  rclcpp_action
  std_msgs 
  std_srvs 
  sensor_msgs 
  wsg_50_common)

install(TARGETS
  wsg_50_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()


