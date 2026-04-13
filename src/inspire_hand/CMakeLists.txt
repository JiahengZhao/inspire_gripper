cmake_minimum_required(VERSION 3.16)
project(inspire_hand LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(SRV_FILES
  srv/Stop.srv
  srv/ClearFault.srv
  srv/SaveParams.srv
  srv/SetId.srv
  srv/CalibrateRange.srv
  srv/SetForceThreshold.srv
  srv/ReadRunState.srv
)
rosidl_generate_interfaces(${PROJECT_NAME} ${SRV_FILES})

add_library(${PROJECT_NAME}_plugin SHARED
  src/protocol.cpp
  src/serial_bus.cpp
  src/gripper_system.cpp
  src/services.cpp
)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp rclcpp_lifecycle hardware_interface pluginlib diagnostic_msgs std_msgs)

rosidl_get_typesupport_target(${PROJECT_NAME}_typesupport
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_plugin ${${PROJECT_NAME}_typesupport})

pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle hardware_interface pluginlib diagnostic_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME}_plugin)

  option(INSPIRE_HAND_HW_TESTS "Build hardware-in-the-loop integration tests" OFF)
  if(INSPIRE_HAND_HW_TESTS)
    ament_add_gtest(test_integration test/test_integration.cpp)
    target_link_libraries(test_integration ${PROJECT_NAME}_plugin)
  endif()
endif()

ament_package()
