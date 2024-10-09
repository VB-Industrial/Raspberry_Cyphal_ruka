cmake_minimum_required(VERSION 3.22)
project(ruka)

find_package(ament_cmake REQUIRED)


if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(PACKAGE_INCLUDE_DEPENDS
   hardware_interface
  controller_interface
  realtime_tools
 # semantic_components
  sensor_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  moveit_ros_planning_interface
)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  ruka
  SHARED
  hardware/ruka_hardware.cpp
  hardware/ruka_sensor_hardware.cpp
 
)

target_compile_features(ruka PUBLIC cxx_std_17)
target_include_directories(ruka PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>

$<INSTALL_INTERFACE:include/ruka>
)
ament_target_dependencies(
  ruka PUBLIC
  ${PACKAGE_INCLUDE_DEPENDS}
 
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RUKA_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface ruka.xml)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})


install(
  DIRECTORY hardware/include/
  DESTINATION include/ruka
)



install(TARGETS ruka
  EXPORT export_ruka
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_ruka HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_INCLUDE_DEPENDS})




ament_package()
