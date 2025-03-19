cmake_minimum_required(VERSION 3.16)
project(wbc_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  realtime_tools
  std_msgs
  geometry_msgs
  wbc_msgs
  robot_control_msgs
  sensor_msgs
  rclcpp_lifecycle
  generate_parameter_library
  rclcpp_components
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include(FindPkgConfig)
pkg_check_modules(wbc-core REQUIRED wbc-core)
pkg_check_modules(wbc-controllers REQUIRED wbc-controllers)
pkg_check_modules(wbc-types REQUIRED wbc-types)
pkg_check_modules(wbc-tasks REQUIRED wbc-tasks)

include_directories(
  ${wbc-core_INCLUDE_DIRS}
  ${wbc-controllers_INCLUDE_DIRS}
  ${wbc-types_INCLUDE_DIRS}
  ${wbc-tasks_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/wbc_ros
)

link_directories(
  ${wbc-core_LIBRARIES}
  ${wbc-controllers_LIBRARIES}
  ${wbc-types_LIBRARIES}
  ${wbc-tasks_LIBRARIES}
)

generate_parameter_library(single_arm_controller_parameters
  src/single_arm_controller_parameters.yaml
)
generate_parameter_library(biped_controller_parameters
  src/biped_controller_parameters.yaml
)

add_library(biped_controller SHARED src/biped_controller.cpp src/conversions.cpp)
add_library(single_arm_controller SHARED src/single_arm_controller.cpp src/conversions.cpp)
add_library(mock_hardware_interface SHARED src/examples/mock_hardware_interface.cpp)
ament_target_dependencies(single_arm_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(biped_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(mock_hardware_interface rclcpp_lifecycle robot_control_msgs sensor_msgs rclcpp_components)

target_link_libraries(single_arm_controller PUBLIC dl ${wbc-core_LIBRARIES} ${wbc-types_LIBRARIES} ${wbc-controllers_LIBRARIES} ${wbc-tasks_LIBRARIES} single_arm_controller_parameters)
target_link_libraries(biped_controller PUBLIC dl ${wbc-core_LIBRARIES} ${wbc-types_LIBRARIES} ${wbc-controllers_LIBRARIES} ${wbc-tasks_LIBRARIES} biped_controller_parameters)
target_compile_features(single_arm_controller PUBLIC cxx_std_17)
target_compile_features(biped_controller PUBLIC cxx_std_17)
target_include_directories(single_arm_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/wbc_ros>
)
target_include_directories(biped_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/wbc_ros>
)

rclcpp_components_register_node(single_arm_controller PLUGIN "wbc_ros::SingleArmController" EXECUTABLE single_arm_controller_node)
rclcpp_components_register_node(biped_controller PLUGIN "wbc_ros::BipedController" EXECUTABLE biped_controller_node)
rclcpp_components_register_node(mock_hardware_interface PLUGIN "wbc_ros::MockHardwareInterface" EXECUTABLE mock_hardware_interface_node)

install(
  DIRECTORY include/
  DESTINATION include/wbc_ros
)
install(TARGETS mock_hardware_interface
                single_arm_controller
                biped_controller
                single_arm_controller_parameters
                biped_controller_parameters
        EXPORT export_wbc_ros
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY models
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_wbc_ros HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

##############
## Examples ##
##############

add_executable(joint_trajectory_publisher src/examples/joint_trajectory_publisher.cpp)
add_executable(cartesian_trajectory_publisher src/examples/cartesian_trajectory_publisher.cpp)
add_executable(cartesian_pose_publisher src/examples/cartesian_pose_publisher.cpp)
ament_target_dependencies(joint_trajectory_publisher rclcpp robot_control_msgs)
ament_target_dependencies(cartesian_trajectory_publisher rclcpp robot_control_msgs)
ament_target_dependencies(cartesian_pose_publisher rclcpp robot_control_msgs)

install(TARGETS joint_trajectory_publisher
                cartesian_trajectory_publisher
                cartesian_pose_publisher
        RUNTIME DESTINATION lib/${PROJECT_NAME})


#############
## Testing ##
#############

if(BUILD_TESTING)
find_package(launch_testing_ament_cmake)
add_launch_test(test/simple_launch.test.py)
install(DIRECTORY test
        DESTINATION share/${PROJECT_NAME})
endif()

#############
## Doxygen ##
#############

find_package(Doxygen)
if(BUILD_DOC)
  if(DOXYGEN_FOUND)
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    configure_file(${doxyfile_in} ${doxyfile} @ONLY)
    message("Doxygen build started.")
    add_custom_target(doc ALL
                      COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
                      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                     COMMENT "Generating API documentation with Doxygen"
                      VERBATIM)
   install(DIRECTORY doc/images DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/doc)
  else(DOXYGEN_FOUND)
     message("Doxygen needs to be installed to generate the doxygen documentation")
  endif(DOXYGEN_FOUND)
endif (BUILD_DOC)
