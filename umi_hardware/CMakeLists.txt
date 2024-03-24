cmake_minimum_required(VERSION 3.16)
project(umi_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  umi_robot_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  umi_hardware
  SHARED
  src/umi_robot_hardware.cpp
  src/umi_executor.cpp
)
target_compile_features(umi_hardware PUBLIC cxx_std_17)
target_include_directories(umi_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/umi_hardware>
)
ament_target_dependencies(
  umi_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "umi_hardware_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface umi_robot_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/umi_hardware
)
install(
  DIRECTORY description rviz
  DESTINATION share/umi_hardware
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/umi_hardware
)
install(TARGETS umi_hardware
  EXPORT export_umi_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_pytest REQUIRED)

#   ament_add_pytest_test(example_1_urdf_xacro test/test_urdf_xacro.py)
#   ament_add_pytest_test(view_example_1_launch test/test_view_robot_launch.py)
# endif()


## EXPORTS
ament_export_targets(export_umi_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()