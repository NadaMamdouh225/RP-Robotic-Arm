cmake_minimum_required(VERSION 3.8)
project(rp_arm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}              #install this in share folder of ros2_ws, PROJECT_NAME = RP_Arm_bringup
)


ament_package()
