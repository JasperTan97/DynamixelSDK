################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare ament specific configuration to be passed to dependent projects
################################################################################
ament_export_dependencies(rclcpp)

################################################################################
# Build
################################################################################
include_directories(
  include/${PROJECT_NAME}
)

if(APPLE)
  add_library(dynamixel_sdk
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_mac.cpp
  )
else()
  add_library(dynamixel_sdk
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_linux.cpp
  )
endif()

ament_target_dependencies(${PROJECT_NAME} rclcpp)
target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

################################################################################
# Install
################################################################################
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

#catkin_install_python(PROGRAMS
#  src/${PROJECT_NAME}/group_bulk_read.py
#  src/${PROJECT_NAME}/group_bulk_write.py
#  src/${PROJECT_NAME}/group_sync_read.py
#  src/${PROJECT_NAME}/group_sync_write.py
#  src/${PROJECT_NAME}/packet_handler.py
#  src/${PROJECT_NAME}/port_handler.py
#  src/${PROJECT_NAME}/protocol1_packet_handler.py
#  src/${PROJECT_NAME}/protocol2_packet_handler.py
#  src/${PROJECT_NAME}/robotis_def.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
