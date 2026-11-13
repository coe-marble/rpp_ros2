cmake_minimum_required(VERSION 3.15)
project(rpp_ros2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python 2.8.8 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rpp_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ComponentStatus.msg"
  "msg/SlotStatus.msg"
  "srv/ConfigureComponent.srv"
  "srv/ActivateComponent.srv"
  "srv/DeactivateComponent.srv"
  "srv/CleanupComponent.srv"
  "srv/LoadComponent.srv"
  "srv/UnloadComponent.srv"
  "srv/ReloadComponent.srv"
  "srv/SwitchComponents.srv"
  "srv/GetSlotStatus.srv"
  "srv/ListComponents.srv"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rosidl owns the target named rpp_ros2. This separate target installs no library.
add_library(rpp_ros2_helpers INTERFACE)
add_library(rpp_ros2::rpp_ros2 ALIAS rpp_ros2_helpers)
set_target_properties(rpp_ros2_helpers PROPERTIES EXPORT_NAME rpp_ros2)
target_compile_features(rpp_ros2_helpers INTERFACE cxx_std_17)
target_include_directories(rpp_ros2_helpers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rpp_ros2_helpers INTERFACE
  rpp_cpp::rpp_cpp
  rclcpp::rclcpp
  "${cpp_typesupport_target}"
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rpp_ros2_helpers EXPORT export_${PROJECT_NAME})
# Lyrical merges this source module with rosidl's generated msg/ and srv/ packages.
ament_python_install_package(${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_slot_manager_ros test/test_slot_manager_ros.py)
  ament_add_pytest_test(test_component_lookup test/test_component_lookup.py)
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rpp_cpp rclcpp std_msgs rosidl_default_runtime)
ament_package()
