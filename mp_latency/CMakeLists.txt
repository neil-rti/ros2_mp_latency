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
target_compile_definitions(ipcsource_1kb PUBLIC MP_DATA_SIZE=1024)
add_executable(ipcwork_1kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_1kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_1kb PUBLIC MP_DATA_SIZE=1024)
add_executable(ipcsink_1kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_1kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_1kb PUBLIC MP_DATA_SIZE=1024)

# stamped 4kb
add_executable(ipcsource_4kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_4kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_4kb PUBLIC MP_DATA_SIZE=4096)
add_executable(ipcwork_4kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_4kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_4kb PUBLIC MP_DATA_SIZE=4096)
add_executable(ipcsink_4kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_4kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_4kb PUBLIC MP_DATA_SIZE=4096)

# stamped 10kb
add_executable(ipcsource_10kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_10kb PUBLIC MP_DATA_SIZE=10240)
add_executable(ipcwork_10kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_10kb PUBLIC MP_DATA_SIZE=10240)
add_executable(ipcsink_10kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_10kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_10kb PUBLIC MP_DATA_SIZE=10240)

# stamped 16kb
add_executable(ipcsource_16kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_16kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_16kb PUBLIC MP_DATA_SIZE=16384)
add_executable(ipcwork_16kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_16kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_16kb PUBLIC MP_DATA_SIZE=16384)
add_executable(ipcsink_16kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_16kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_16kb PUBLIC MP_DATA_SIZE=16384)

# stamped 32kb
add_executable(ipcsource_32kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_32kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_32kb PUBLIC MP_DATA_SIZE=32768)
add_executable(ipcwork_32kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_32kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_32kb PUBLIC MP_DATA_SIZE=32768)
add_executable(ipcsink_32kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_32kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_32kb PUBLIC MP_DATA_SIZE=32768)

# stamped 60kb
add_executable(ipcsource_60kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_60kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_60kb PUBLIC MP_DATA_SIZE=61440)
add_executable(ipcwork_60kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_60kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_60kb PUBLIC MP_DATA_SIZE=61440)
add_executable(ipcsink_60kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_60kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_60kb PUBLIC MP_DATA_SIZE=61440)

# stamped 100kb
add_executable(ipcsource_100kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_100kb PUBLIC MP_DATA_SIZE=102400)
add_executable(ipcwork_100kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_100kb PUBLIC MP_DATA_SIZE=102400)
add_executable(ipcsink_100kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_100kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_100kb PUBLIC MP_DATA_SIZE=102400)

# stamped 256kb
add_executable(ipcsource_256kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_256kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_256kb PUBLIC MP_DATA_SIZE=262144)
add_executable(ipcwork_256kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_256kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_256kb PUBLIC MP_DATA_SIZE=262144)
add_executable(ipcsink_256kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_256kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_256kb PUBLIC MP_DATA_SIZE=262144)

# stamped 500kb
add_executable(ipcsource_500kb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_500kb PUBLIC MP_DATA_SIZE=512000)
add_executable(ipcwork_500kb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_500kb PUBLIC MP_DATA_SIZE=512000)
add_executable(ipcsink_500kb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_500kb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_500kb PUBLIC MP_DATA_SIZE=512000)

# stamped 1mb
add_executable(ipcsource_1mb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_1mb PUBLIC MP_DATA_SIZE=1048576)
add_executable(ipcwork_1mb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_1mb PUBLIC MP_DATA_SIZE=1048576)
add_executable(ipcsink_1mb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_1mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_1mb PUBLIC MP_DATA_SIZE=1048576)

# stamped 2mb
add_executable(ipcsource_2mb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_2mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_2mb PUBLIC MP_DATA_SIZE=2097152)
add_executable(ipcwork_2mb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_2mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_2mb PUBLIC MP_DATA_SIZE=2097152)
add_executable(ipcsink_2mb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_2mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_2mb PUBLIC MP_DATA_SIZE=2097152)

# stamped 4mb
add_executable(ipcsource_4mb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_4mb PUBLIC MP_DATA_SIZE=4194304)
add_executable(ipcwork_4mb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_4mb PUBLIC MP_DATA_SIZE=4194304)
add_executable(ipcsink_4mb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_4mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_4mb PUBLIC MP_DATA_SIZE=4194304)

# stamped 8mb
add_executable(ipcsource_8mb src/IpcSource.cpp)
ament_target_dependencies(ipcsource_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsource_8mb PUBLIC MP_DATA_SIZE=8388608)
add_executable(ipcwork_8mb src/IpcWork.cpp)
ament_target_dependencies(ipcwork_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcwork_8mb PUBLIC MP_DATA_SIZE=8388608)
add_executable(ipcsink_8mb src/IpcSink.cpp)
ament_target_dependencies(ipcsink_8mb rclcpp irobot_interfaces_plugin performance_test_msgs builtin_interfaces)
target_compile_definitions(ipcsink_8mb PUBLIC MP_DATA_SIZE=8388608)


install(TARGETS 
  ipcsource_100b   ipcwork_100b   ipcsink_100b
  ipcsource_1kb    ipcwork_1kb    ipcsink_1kb
  ipcsource_4kb    ipcwork_4kb    ipcsink_4kb
  ipcsource_10kb   ipcwork_10kb   ipcsink_10kb
  ipcsource_16kb   ipcwork_16kb   ipcsink_16kb
  ipcsource_32kb   ipcwork_32kb   ipcsink_32kb
  ipcsource_60kb   ipcwork_60kb   ipcsink_60kb
  ipcsource_100kb  ipcwork_100kb  ipcsink_100kb
  ipcsource_256kb  ipcwork_256kb  ipcsink_256kb
  ipcsource_500kb  ipcwork_500kb  ipcsink_500kb
  ipcsource_1mb    ipcwork_1mb    ipcsink_1mb
  ipcsource_2mb    ipcwork_2mb    ipcsink_2mb
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
