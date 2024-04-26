cmake_minimum_required(VERSION 3.22)
project(ruka)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)  

if(DEFINED ENV{<CYPHAL_DIR>})
    set(CYPHAL_DIR $ENV{CYPHAL_DIR})
else()
    get_filename_component(CYPHAL_DIR
                           "../libcxxcanard"
                           ABSOLUTE)
endif()
message(STATUS "${CMAKE_CURRENT_LIST_DIR} | Using <${CYPHAL_DIR}> as directory of libcxxcanard")

add_subdirectory(${CYPHAL_DIR} ${PROJECT_BINARY_DIR}/build/libcxxcanard)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  ruka
  SHARED
  hardware/ruka_hardware.cpp
)

target_include_directories(ruka PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
#${CYPHAL_DIR}
#${CYPHAL_DIR}/libs
#$<INSTALL_INTERFACE:include/ruka>
)

target_include_directories(ruka PUBLIC
${CYPHAL_DIR}
${COMMON_LIBS_DIR}
#$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/drivers/chyphal/libs
)

target_link_libraries(ruka PUBLIC libcxxcanard)

ament_target_dependencies(
  ruka PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "RUKA_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface ruka.xml)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})


install(
  DIRECTORY hardware/include/
  DESTINATION include/ruka
)

install(TARGETS libcxxcanard
 EXPORT export_libcxxcanard 
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ruka
  EXPORT export_ruka
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


## EXPORTS
ament_export_targets(export_ruka HAS_LIBRARY_TARGET)
ament_export_targets(export_libcxxcanard HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
