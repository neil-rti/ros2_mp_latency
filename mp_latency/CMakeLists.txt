cmake_minimum_required(VERSION 3.5)
project(mp_latency)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(irobot_interfaces_plugin REQUIRED)
find_package(performance_test_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include)

add_executable(ipchead src/IpcHead.cpp)
ament_target_dependencies(ipchead rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)

add_executable(ipcnode src/IpcNode.cpp)
ament_target_dependencies(ipcnode rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)

add_executable(ipctail src/IpcTail.cpp)
ament_target_dependencies(ipctail rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)

install(TARGETS 
  ipchead
  ipcnode
  ipctail
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
