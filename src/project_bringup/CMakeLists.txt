cmake_minimum_required(VERSION 3.8)
project(project_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Correct installation of launch directory
install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY rviz/ 
  DESTINATION share/${PROJECT_NAME}/rviz)

install(DIRECTORY config/ 
  DESTINATION share/${PROJECT_NAME}/config)
ament_package()