cmake_minimum_required(VERSION 3.10)
project(robstride_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

if(hardware_interface_VERSION VERSION_GREATER_EQUAL "5.0.0")
  set(ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS ON)
endif()

add_library(${PROJECT_NAME} SHARED
  src/driver/command_mode.cpp
  src/driver/driver_config.cpp
  src/driver/robstride_driver.cpp
  src/protocol/protocol.cpp
  src/robstride_system.cpp
  src/transport/can_transport.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
if(ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS)
  target_compile_definitions(${PROJECT_NAME}
    PUBLIC ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS
  )
endif()
if(COMMAND ament_target_dependencies)
  # Humble through Kilted provide this compatibility macro.
  ament_target_dependencies(${PROJECT_NAME}
    can_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle
  )
else()
  # The macro was removed after being deprecated in Kilted.  Use the modern
  # imported targets on Lyrical and Rolling.
  target_link_libraries(${PROJECT_NAME} PUBLIC
    ${can_msgs_TARGETS}
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  )
endif()
pluginlib_export_plugin_description_file(hardware_interface robstride_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES include/robstride_ros2/robstride_system.hpp
  DESTINATION include/robstride_ros2
)
install(DIRECTORY config description launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp src/protocol/protocol.cpp)
  target_include_directories(test_protocol PRIVATE src)
  ament_add_gtest(test_command_mode test/test_command_mode.cpp src/driver/command_mode.cpp)
  target_include_directories(test_command_mode PRIVATE src)
  ament_add_gtest(test_can_transport test/test_can_transport.cpp)
  target_include_directories(test_can_transport PRIVATE src)
  target_link_libraries(test_can_transport ${PROJECT_NAME})
  ament_add_gtest(test_joint_data test/test_joint_data.cpp)
  target_include_directories(test_joint_data PRIVATE src)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(can_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
