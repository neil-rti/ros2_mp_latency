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
add_executable(ipcsource_100b src/IpcSource.cpp)
ament_target_dependencies(ipcsource_100b rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_100b PUBLIC MP_DATA_SIZE=100)
add_executable(ipcprocess_100b src/IpcProcess.cpp)
ament_target_dependencies(ipcprocess_100b rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcprocess_100b PUBLIC MP_DATA_SIZE=100)
add_executable(ipcsink_100b src/IpcSink.cpp)
ament_target_dependencies(ipcsink_100b rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_100b PUBLIC MP_DATA_SIZE=100)

# stamped 1kb
add_executable(ipcsource_1kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_1kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_1kb PUBLIC MP_DATA_SIZE=1000)
add_executable(ipcprocess_1kb src/IpcProcess.cpp)
ament_target_dependencies(ipcprocess_1kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcprocess_1kb PUBLIC MP_DATA_SIZE=1000)
add_executable(ipcsink_1kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_1kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_1kb PUBLIC MP_DATA_SIZE=1000)

# stamped 10kb
add_executable(ipcsource_10kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_10kb PUBLIC MP_DATA_SIZE=10000)
add_executable(ipcprocess_10kb src/IpcProcess.cpp)
ament_target_dependencies(ipcprocess_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcprocess_10kb PUBLIC MP_DATA_SIZE=10000)
add_executable(ipcsink_10kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_10kb PUBLIC MP_DATA_SIZE=10000)

# stamped 100kb
add_executable(ipcsource_100kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_100kb PUBLIC MP_DATA_SIZE=100000)
add_executable(ipcprocess_100kb src/IpcProcess.cpp)
ament_target_dependencies(ipcprocess_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcprocess_100kb PUBLIC MP_DATA_SIZE=100000)
add_executable(ipcsink_100kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_100kb PUBLIC MP_DATA_SIZE=100000)

# stamped 500kb
add_executable(ipcsource_500kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_500kb PUBLIC MP_DATA_SIZE=500000)
add_executable(ipcprocess_500kb src/IpcProcess.cpp)
ament_target_dependencies(ipcprocess_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcprocess_500kb PUBLIC MP_DATA_SIZE=500000)
add_executable(ipcsink_500kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_500kb PUBLIC MP_DATA_SIZE=500000)

# stamped 1mb
add_executable(ipcsource_1mb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_1mb PUBLIC MP_DATA_SIZE=1000000)
add_executable(ipcprocess_1mb src/IpcProcess.cpp)
ament_target_dependencies(ipcprocess_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcprocess_1mb PUBLIC MP_DATA_SIZE=1000000)
add_executable(ipcsink_1mb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_1mb PUBLIC MP_DATA_SIZE=1000000)

# stamped 4mb
add_executable(ipcsource_4mb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_4mb PUBLIC MP_DATA_SIZE=4000000)
add_executable(ipcprocess_4mb src/IpcProcess.cpp)
ament_target_dependencies(ipcprocess_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcprocess_4mb PUBLIC MP_DATA_SIZE=4000000)
add_executable(ipcsink_4mb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_4mb PUBLIC MP_DATA_SIZE=4000000)

# stamped 8mb
add_executable(ipcsource_8mb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_8mb PUBLIC MP_DATA_SIZE=8000000)
add_executable(ipcprocess_8mb src/IpcProcess.cpp)
ament_target_dependencies(ipcprocess_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcprocess_8mb PUBLIC MP_DATA_SIZE=8000000)
add_executable(ipcsink_8mb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_8mb PUBLIC MP_DATA_SIZE=8000000)


install(TARGETS 
  ipcsource_100b   ipcprocess_100b   ipcsink_100b
  ipcsource_1kb    ipcprocess_1kb    ipcsink_1kb
  ipcsource_10kb   ipcprocess_10kb   ipcsink_10kb
  ipcsource_100kb  ipcprocess_100kb  ipcsink_100kb
  ipcsource_500kb  ipcprocess_500kb  ipcsink_500kb
  ipcsource_1mb    ipcprocess_1mb    ipcsink_1mb
  ipcsource_4mb    ipcprocess_4mb    ipcsink_4mb
  ipcsource_8mb    ipcprocess_8mb    ipcsink_8mb
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
