cmake_minimum_required(VERSION 3.8)
project(basic_mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config launch  models src worlds maps demos description
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    basic_mobile_robot/follow.py
    basic_mobile_robot/viewer.py
    basic_mobile_robot/follow_line.py
    DESTINATION lib/${PROJECT_NAME}
    )


ament_package()
