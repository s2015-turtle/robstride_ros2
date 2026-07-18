cmake_minimum_required(VERSION 3.10)
project(robstride_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_target_dependencies REQUIRED)
find_package(can_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

if(hardware_interface_VERSION VERSION_GREATER_EQUAL "5.0.0")
  set(ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS ON)
endif()

add_library(${PROJECT_NAME} SHARED
  src/command_mode.cpp
  src/protocol.cpp
  src/robstride_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS)
  target_compile_definitions(${PROJECT_NAME}
    PUBLIC ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS
  )
endif()
ament_target_dependencies(${PROJECT_NAME}
  can_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface robstride_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config description launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp src/protocol.cpp)
  target_include_directories(test_protocol PRIVATE include)
  ament_add_gtest(test_command_mode test/test_command_mode.cpp src/command_mode.cpp)
  target_include_directories(test_command_mode PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(can_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
