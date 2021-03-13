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

# create separate executables for SOURCE/WORK/SINK at each data size
# stamped 100b
add_executable(ipcsource_100b src/IpcSource.cpp)
ament_target_dependencies(ipcsource_100b rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_100b PUBLIC MP_DATA_SIZE=100)
add_executable(ipcwork_100b src/IpcWork.cpp)
ament_target_dependencies(ipcwork_100b rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_100b PUBLIC MP_DATA_SIZE=100)
add_executable(ipcsink_100b src/IpcSink.cpp)
ament_target_dependencies(ipcsink_100b rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_100b PUBLIC MP_DATA_SIZE=100)

# stamped 1kb
add_executable(ipcsource_1kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_1kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_1kb PUBLIC MP_DATA_SIZE=1000)
add_executable(ipcwork_1kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_1kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_1kb PUBLIC MP_DATA_SIZE=1000)
add_executable(ipcsink_1kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_1kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_1kb PUBLIC MP_DATA_SIZE=1000)

# stamped 10kb
add_executable(ipcsource_10kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_10kb PUBLIC MP_DATA_SIZE=10000)
add_executable(ipcwork_10kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_10kb PUBLIC MP_DATA_SIZE=10000)
add_executable(ipcsink_10kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_10kb PUBLIC MP_DATA_SIZE=10000)

# stamped 100kb
add_executable(ipcsource_100kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_100kb PUBLIC MP_DATA_SIZE=100000)
add_executable(ipcwork_100kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_100kb PUBLIC MP_DATA_SIZE=100000)
add_executable(ipcsink_100kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_100kb PUBLIC MP_DATA_SIZE=100000)

# stamped 500kb
add_executable(ipcsource_500kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_500kb PUBLIC MP_DATA_SIZE=500000)
add_executable(ipcwork_500kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_500kb PUBLIC MP_DATA_SIZE=500000)
add_executable(ipcsink_500kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_500kb PUBLIC MP_DATA_SIZE=500000)

# stamped 1mb
add_executable(ipcsource_1mb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_1mb PUBLIC MP_DATA_SIZE=1000000)
add_executable(ipcwork_1mb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_1mb PUBLIC MP_DATA_SIZE=1000000)
add_executable(ipcsink_1mb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_1mb PUBLIC MP_DATA_SIZE=1000000)

# stamped 4mb
add_executable(ipcsource_4mb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_4mb PUBLIC MP_DATA_SIZE=4000000)
add_executable(ipcwork_4mb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_4mb PUBLIC MP_DATA_SIZE=4000000)
add_executable(ipcsink_4mb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_4mb PUBLIC MP_DATA_SIZE=4000000)

# stamped 8mb
add_executable(ipcsource_8mb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_8mb PUBLIC MP_DATA_SIZE=8000000)
add_executable(ipcwork_8mb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_8mb PUBLIC MP_DATA_SIZE=8000000)
add_executable(ipcsink_8mb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_8mb PUBLIC MP_DATA_SIZE=8000000)


install(TARGETS 
  ipcsource_100b   ipcwork_100b   ipcsink_100b
  ipcsource_1kb    ipcwork_1kb    ipcsink_1kb
  ipcsource_10kb   ipcwork_10kb   ipcsink_10kb
  ipcsource_100kb  ipcwork_100kb  ipcsink_100kb
  ipcsource_500kb  ipcwork_500kb  ipcsink_500kb
  ipcsource_1mb    ipcwork_1mb    ipcsink_1mb
  ipcsource_4mb    ipcwork_4mb    ipcsink_4mb
  ipcsource_8mb    ipcwork_8mb    ipcsink_8mb
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
