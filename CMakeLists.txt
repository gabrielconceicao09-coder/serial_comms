cmake_minimum_required(VERSION 3.20)
project(serial_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include
	include/serial_comms
	src
)
link_directories(
	include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(serial_imu_node
	src/serial_imu_node.cpp
	src/serial.cc
	src/impl/unix.cc
	src/impl/win.cc
	src/impl/list_ports/list_ports_linux.cc
	src/impl/list_ports/list_ports_osx.cc
	src/impl/list_ports/list_ports_win.cc
)
##target_link_libraries(serial_imu_node PUBLIC #para rodar no foxy
##rclcpp::rclcpp
##sensor_msgs
##)
ament_target_dependencies(serial_imu_node #para rodar no kilted
rclcpp
sensor_msgs
) 

install(TARGETS serial_imu_node
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
