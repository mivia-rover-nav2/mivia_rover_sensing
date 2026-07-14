cmake_minimum_required(VERSION 3.16)
project(dwm1001_driver_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(dummy_active_tag src/dummy_active_tag_node.cpp)
target_compile_features(dummy_active_tag PUBLIC cxx_std_17)
ament_target_dependencies(dummy_active_tag rclcpp rcl_interfaces geometry_msgs)

add_executable(dummy_passive_tag src/dummy_passive_tag_node.cpp)
target_compile_features(dummy_passive_tag PUBLIC cxx_std_17)
ament_target_dependencies(dummy_passive_tag rclcpp rcl_interfaces geometry_msgs)

install(TARGETS dummy_active_tag dummy_passive_tag
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
