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

# create separate executables for HEAD/THRU/TAIL at each data size
# stamped 100b
add_executable(ipchead_100b src/IpcHead.cpp)
ament_target_dependencies(ipchead_100b rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipchead_100b PUBLIC MP_DATA_SIZE=100)
add_executable(ipcthru_100b src/IpcThru.cpp)
ament_target_dependencies(ipcthru_100b rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcthru_100b PUBLIC MP_DATA_SIZE=100)
add_executable(ipctail_100b src/IpcTail.cpp)
ament_target_dependencies(ipctail_100b rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipctail_100b PUBLIC MP_DATA_SIZE=100)

# stamped 1kb
add_executable(ipchead_1kb src/IpcHead.cpp)
ament_target_dependencies(ipchead_1kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipchead_1kb PUBLIC MP_DATA_SIZE=1000)
add_executable(ipcthru_1kb src/IpcThru.cpp)
ament_target_dependencies(ipcthru_1kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcthru_1kb PUBLIC MP_DATA_SIZE=1000)
add_executable(ipctail_1kb src/IpcTail.cpp)
ament_target_dependencies(ipctail_1kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipctail_1kb PUBLIC MP_DATA_SIZE=1000)

# stamped 10kb
add_executable(ipchead_10kb src/IpcHead.cpp)
ament_target_dependencies(ipchead_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipchead_10kb PUBLIC MP_DATA_SIZE=10000)
add_executable(ipcthru_10kb src/IpcThru.cpp)
ament_target_dependencies(ipcthru_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcthru_10kb PUBLIC MP_DATA_SIZE=10000)
add_executable(ipctail_10kb src/IpcTail.cpp)
ament_target_dependencies(ipctail_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipctail_10kb PUBLIC MP_DATA_SIZE=10000)

# stamped 100kb
add_executable(ipchead_100kb src/IpcHead.cpp)
ament_target_dependencies(ipchead_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipchead_100kb PUBLIC MP_DATA_SIZE=100000)
add_executable(ipcthru_100kb src/IpcThru.cpp)
ament_target_dependencies(ipcthru_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcthru_100kb PUBLIC MP_DATA_SIZE=100000)
add_executable(ipctail_100kb src/IpcTail.cpp)
ament_target_dependencies(ipctail_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipctail_100kb PUBLIC MP_DATA_SIZE=100000)

# stamped 500kb
add_executable(ipchead_500kb src/IpcHead.cpp)
ament_target_dependencies(ipchead_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipchead_500kb PUBLIC MP_DATA_SIZE=500000)
add_executable(ipcthru_500kb src/IpcThru.cpp)
ament_target_dependencies(ipcthru_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcthru_500kb PUBLIC MP_DATA_SIZE=500000)
add_executable(ipctail_500kb src/IpcTail.cpp)
ament_target_dependencies(ipctail_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipctail_500kb PUBLIC MP_DATA_SIZE=500000)

# stamped 1mb
add_executable(ipchead_1mb src/IpcHead.cpp)
ament_target_dependencies(ipchead_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipchead_1mb PUBLIC MP_DATA_SIZE=1000000)
add_executable(ipcthru_1mb src/IpcThru.cpp)
ament_target_dependencies(ipcthru_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcthru_1mb PUBLIC MP_DATA_SIZE=1000000)
add_executable(ipctail_1mb src/IpcTail.cpp)
ament_target_dependencies(ipctail_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipctail_1mb PUBLIC MP_DATA_SIZE=1000000)

# stamped 4mb
add_executable(ipchead_4mb src/IpcHead.cpp)
ament_target_dependencies(ipchead_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipchead_4mb PUBLIC MP_DATA_SIZE=4000000)
add_executable(ipcthru_4mb src/IpcThru.cpp)
ament_target_dependencies(ipcthru_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcthru_4mb PUBLIC MP_DATA_SIZE=4000000)
add_executable(ipctail_4mb src/IpcTail.cpp)
ament_target_dependencies(ipctail_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipctail_4mb PUBLIC MP_DATA_SIZE=4000000)

# stamped 8mb
add_executable(ipchead_8mb src/IpcHead.cpp)
ament_target_dependencies(ipchead_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipchead_8mb PUBLIC MP_DATA_SIZE=8000000)
add_executable(ipcthru_8mb src/IpcThru.cpp)
ament_target_dependencies(ipcthru_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcthru_8mb PUBLIC MP_DATA_SIZE=8000000)
add_executable(ipctail_8mb src/IpcTail.cpp)
ament_target_dependencies(ipctail_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipctail_8mb PUBLIC MP_DATA_SIZE=8000000)


install(TARGETS 
  ipchead_100b   ipcthru_100b   ipctail_100b
  ipchead_1kb    ipcthru_1kb    ipctail_1kb
  ipchead_10kb   ipcthru_10kb   ipctail_10kb
  ipchead_100kb  ipcthru_100kb  ipctail_100kb
  ipchead_500kb  ipcthru_500kb  ipctail_500kb
  ipchead_1mb    ipcthru_1mb    ipctail_1mb
  ipchead_4mb    ipcthru_4mb    ipctail_4mb
  ipchead_8mb    ipcthru_8mb    ipctail_8mb
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
