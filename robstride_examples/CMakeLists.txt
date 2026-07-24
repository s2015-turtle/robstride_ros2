cmake_minimum_required(VERSION 3.10)
project(robstride_examples)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config description launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_motor_profile_gains test/test_motor_profile_gains.py)
endif()

ament_package()
