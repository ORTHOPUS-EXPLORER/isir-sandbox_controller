cmake_minimum_required(VERSION 3.8)
project(sandbox_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Add all dependencies that are needed
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(extender_msgs REQUIRED)

generate_parameter_library(
  sandbox_param_lib
  src/sandbox_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/sandbox_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  sandbox_param_lib
  ${cpp_typesupport_target}
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  controller_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  std_msgs
  std_srvs
  geometry_msgs
  Eigen3
  robot_interfaces
  extender_msgs
)

pluginlib_export_plugin_description_file(
        controller_interface src/sandbox_controller.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_sandbox_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY bringup/config bringup/launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  controller_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  std_msgs
  Eigen3
  robot_interfaces
  extender_msgs
)

ament_package()