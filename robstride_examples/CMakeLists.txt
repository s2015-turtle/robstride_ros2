cmake_minimum_required(VERSION 3.10)
project(robstride_examples)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config description launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
