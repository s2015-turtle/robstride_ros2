cmake_minimum_required(VERSION 3.10)
project(robstride_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/can_transport.cpp
  src/driver.cpp
  src/protocol.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(${PROJECT_NAME} can_msgs rclcpp)
  target_link_libraries(${PROJECT_NAME} Threads::Threads)
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC
    ${can_msgs_TARGETS}
    rclcpp::rclcpp
  )
  target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)
endif()

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
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME})
  ament_add_gtest(test_can_transport test/test_can_transport.cpp)
  target_link_libraries(test_can_transport ${PROJECT_NAME})
  ament_add_gtest(test_joint_data test/test_joint_data.cpp)
  target_link_libraries(test_joint_data ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE ${PROJECT_NAME}::)
ament_export_dependencies(can_msgs rclcpp)
ament_package()
