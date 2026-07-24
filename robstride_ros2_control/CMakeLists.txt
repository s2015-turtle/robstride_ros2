cmake_minimum_required(VERSION 3.10)
project(robstride_ros2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robstride_driver REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/command_mode.cpp
  src/driver_config.cpp
  src/robstride_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/internal)
if(hardware_interface_VERSION VERSION_GREATER_EQUAL "5.0.0")
  target_compile_definitions(${PROJECT_NAME}
    PUBLIC ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS
  )
endif()
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(${PROJECT_NAME}
    hardware_interface pluginlib rclcpp rclcpp_lifecycle
  )
  target_link_libraries(${PROJECT_NAME} robstride_driver::robstride_driver)
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    robstride_driver::robstride_driver
  )
endif()
pluginlib_export_plugin_description_file(hardware_interface robstride_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_mode test/test_command_mode.cpp src/command_mode.cpp)
  target_include_directories(test_command_mode PRIVATE internal)
  ament_add_gtest(test_command_limit_config test/test_command_limit_config.cpp)
  target_include_directories(test_command_limit_config PRIVATE internal)
  target_link_libraries(test_command_limit_config ${PROJECT_NAME})
  ament_add_gtest(test_driver_config test/test_driver_config.cpp)
  target_include_directories(test_driver_config PRIVATE internal)
  target_link_libraries(test_driver_config ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle robstride_driver
)
ament_package()
