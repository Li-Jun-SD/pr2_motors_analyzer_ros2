cmake_minimum_required(VERSION 3.8)
project(pr2_motors_analyzer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_aggregator REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# You should add a "SHARED" here, otherwise .so would not generate
# and plugin would not be loaded for library not found.
add_library(pr2_motors_analyzer SHARED
  src/pr2_motors_analyzer.cpp)
target_compile_features(pr2_motors_analyzer PUBLIC c_std_99 cxx_std_17)
target_include_directories(pr2_motors_analyzer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pr2_motors_analyzer
  diagnostic_aggregator
  pluginlib
  diagnostic_msgs
  rclcpp
)

target_compile_definitions(pr2_motors_analyzer PRIVATE "PR2_MOTORS_ANALYZER")

pluginlib_export_plugin_description_file(diagnostic_aggregator pr2_motors_analyer_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include 
)

install(
  TARGETS pr2_motors_analyzer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  pr2_motors_analyzer
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/pr2_power_pub.py
  scripts/pr2_power_pub_estop_hit.py
  DESTINATION lib/${PROJECT_NAME})

  # Should not write a /launch after ${PROJECT_NAME}
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
