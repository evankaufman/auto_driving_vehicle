cmake_minimum_required(VERSION 3.8)
project(mpc_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mpc_ctrl_node src/mpc_ctrl_node.cpp)
target_link_libraries(mpc_ctrl_node PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

install(TARGETS mpc_ctrl_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
