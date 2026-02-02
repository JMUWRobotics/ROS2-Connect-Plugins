cmake_minimum_required(VERSION 3.8)
project(connect_plugins)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(connect REQUIRED)

find_package(example_interfaces REQUIRED)

pluginlib_export_plugin_description_file(connect plugins.xml)

add_library(connect_plugins
        src/action/fibonacciActionClient.cpp
        src/action/fibonacciActionServer.cpp
        src/service/addTwoIntsServiceClient.cpp
        src/service/addTwoIntsServiceServer.cpp)
add_library(connect_plugins::connect_plugins ALIAS connect_plugins)
target_compile_features(connect_plugins PUBLIC c_std_99 cxx_std_23)  # Require C99 and C++23 (original was C++17)
target_include_directories(connect_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  connect_plugins
  "connect"
  "pluginlib"
  "rclcpp"
  "rclcpp_action"
  "example_interfaces"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(connect_plugins PRIVATE "CONNECT_PLUGINS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS connect_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  connect_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
