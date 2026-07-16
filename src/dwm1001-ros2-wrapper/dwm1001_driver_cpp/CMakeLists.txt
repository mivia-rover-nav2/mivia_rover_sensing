cmake_minimum_required(VERSION 3.16)
project(dwm1001_driver_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dwm1001_msg REQUIRED)

add_executable(dummy_active_tag src/dummy_active_tag_node.cpp)
target_compile_features(dummy_active_tag PUBLIC cxx_std_17)
ament_target_dependencies(dummy_active_tag rclcpp rcl_interfaces geometry_msgs)

add_executable(dummy_passive_tag src/dummy_passive_tag_node.cpp)
target_compile_features(dummy_passive_tag PUBLIC cxx_std_17)
ament_target_dependencies(dummy_passive_tag rclcpp rcl_interfaces geometry_msgs)

# Shared serial/shell primitives, reused by the nodes that talk to the
# DWM1001 firmware over UART (les_mode_tag now, ranging_tag/active_tag/
# passive_tag in later phases).
add_library(dwm1001_serial STATIC
  src/serial_port.cpp
  src/dwm1001_shell.cpp
  src/les_line_parser.cpp
  src/pos_report_parser.cpp
)
target_compile_features(dwm1001_serial PUBLIC cxx_std_17)
target_include_directories(dwm1001_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(les_mode_tag src/les_mode_tag_node.cpp)
target_compile_features(les_mode_tag PUBLIC cxx_std_17)
target_link_libraries(les_mode_tag dwm1001_serial)
ament_target_dependencies(les_mode_tag rclcpp rcl_interfaces geometry_msgs dwm1001_msg)

add_executable(ranging_tag src/ranging_tag_node.cpp)
target_compile_features(ranging_tag PUBLIC cxx_std_17)
target_link_libraries(ranging_tag dwm1001_serial)
ament_target_dependencies(ranging_tag rclcpp rcl_interfaces geometry_msgs std_msgs)

add_executable(active_tag src/active_tag_node.cpp)
target_compile_features(active_tag PUBLIC cxx_std_17)
target_link_libraries(active_tag dwm1001_serial)
ament_target_dependencies(active_tag rclcpp rcl_interfaces geometry_msgs dwm1001_msg)

add_executable(passive_tag src/passive_tag_node.cpp)
target_compile_features(passive_tag PUBLIC cxx_std_17)
target_link_libraries(passive_tag dwm1001_serial)
ament_target_dependencies(passive_tag rclcpp rcl_interfaces geometry_msgs)

install(TARGETS dummy_active_tag dummy_passive_tag les_mode_tag ranging_tag active_tag passive_tag
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
