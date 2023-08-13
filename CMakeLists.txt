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
  trajectory_msgs
  wbc_msgs
  sensor_msgs
  pluginlib
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  generate_parameter_library
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include(FindPkgConfig)
pkg_check_modules(wbc-core REQUIRED wbc-core)
pkg_check_modules(wbc-controllers REQUIRED wbc-controllers)

include_directories(
  ${wbc-core_INCLUDE_DIRS}
  ${wbc-controllers_INCLUDE_DIRS}
)

link_directories(
  ${wbc-core_LIBRARIES}
  ${wbc-controllers_LIBRARIES}
)

generate_parameter_library(whole_body_controller_parameters
  src/whole_body_controller_parameters.yaml
)

add_library(wbc_ros SHARED src/whole_body_controller.cpp src/conversions.cpp
                           src/cartesian_position_controller.cpp src/conversions.cpp)
#add_library(loop_back_driver_node SHARED src/loop_back_driver_node.cpp)
#add_library(cartesian_position_controller_node SHARED src/controllers/cartesian_position_controller_node.cpp src/controllers/controller_node.cpp src/conversions.cpp)
#add_library(joint_position_controller_node SHARED src/controllers/joint_position_controller_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)
#add_library(cartesian_force_controller_node SHARED src/controllers/cartesian_force_controller_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)
#add_library(cartesian_radial_potential_fields_node SHARED src/controllers/cartesian_radial_potential_fields_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)
#add_library(joint_limit_avoidance_node SHARED src/controllers/joint_limit_avoidance_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)

ament_target_dependencies(wbc_ros PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
#ament_target_dependencies(loop_back_driver_node rclcpp rclcpp_components trajectory_msgs sensor_msgs)
#ament_target_dependencies(cartesian_position_controller_node rclcpp rclcpp_components wbc_msgs sensor_msgs trajectory_msgs)
#ament_target_dependencies(joint_position_controller_node rclcpp rclcpp_components wbc_msgs sensor_msgs trajectory_msgs)
#ament_target_dependencies(cartesian_force_controller_node rclcpp rclcpp_components wbc_msgs sensor_msgs trajectory_msgs geometry_msgs)
#ament_target_dependencies(cartesian_radial_potential_fields_node rclcpp_components rclcpp wbc_msgs sensor_msgs trajectory_msgs)
#ament_target_dependencies(joint_limit_avoidance_node rclcpp rclcpp_components wbc_msgs sensor_msgs trajectory_msgs)

target_link_libraries(wbc_ros PUBLIC dl ${wbc-core_LIBRARIES})
target_compile_features(wbc_ros PUBLIC cxx_std_17)
target_include_directories(wbc_ros PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/wbc_ros>
)
target_link_libraries(wbc_ros PUBLIC
  whole_body_controller_parameters
)
pluginlib_export_plugin_description_file(controller_interface wbc_plugins.xml)

#target_link_libraries(cartesian_position_controller_node
#   ${wbc-controllers_LIBRARIES}
#)
#target_link_libraries(joint_position_controller_node
#   ${wbc-controllers_LIBRARIES}
#)
#target_link_libraries(cartesian_force_controller_node
#   ${wbc-controllers_LIBRARIES}
#)
#target_link_libraries(cartesian_radial_potential_fields_node
#    ${wbc-controllers_LIBRARIES}
#)
#target_link_libraries(joint_limit_avoidance_node
#    ${wbc-controllers_LIBRARIES}
#)

#rclcpp_components_register_node(wbc_node PLUGIN "wbc_ros::WbcNode" EXECUTABLE wbc)
#rclcpp_components_register_node(loop_back_driver_node PLUGIN "wbc_ros::LoopBackDriverNode" EXECUTABLE loop_back_driver)
#rclcpp_components_register_node(cartesian_position_controller_node PLUGIN "wbc_ros::CartesianPositionControllerNode" EXECUTABLE cartesian_position_controller)
#rclcpp_components_register_node(joint_position_controller_node PLUGIN "wbc_ros::JointPositionControllerNode" EXECUTABLE joint_position_controller)
#rclcpp_components_register_node(cartesian_force_controller_node PLUGIN "wbc_ros::CartesianForceControllerNode" EXECUTABLE cartesian_force_controller)
#rclcpp_components_register_node(cartesian_radial_potential_fields_node PLUGIN "wbc_ros::CartesianRadialPotentialFieldsNode" EXECUTABLE cartesian_radial_potential_fields)
#rclcpp_components_register_node(joint_limit_avoidance_node PLUGIN "wbc_ros::JointLimitAvoidanceNode" EXECUTABLE joint_limit_avoidance)#

install(
  DIRECTORY include/
  DESTINATION include/wbc_ros
)
install(TARGETS wbc_ros
                whole_body_controller_parameters
                #loop_back_driver_node
                #cartesian_position_controller_node
                #joint_position_controller_node
                #cartesian_force_controller_node
                #cartesian_radial_potential_fields_node
                #joint_limit_avoidance_node
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

#add_executable(joint_trajectory_publisher src/examples/joint_trajectory_publisher.cpp)
#add_executable(cartesian_trajectory_publisher src/examples/cartesian_trajectory_publisher.cpp)
#add_executable(cartesian_pose_publisher src/examples/cartesian_pose_publisher.cpp)
#ament_target_dependencies(joint_trajectory_publisher rclcpp trajectory_msgs)
#ament_target_dependencies(cartesian_trajectory_publisher rclcpp wbc_msgs)
#ament_target_dependencies(cartesian_pose_publisher rclcpp wbc_msgs)
#install(TARGETS joint_trajectory_publisher
#                cartesian_trajectory_publisher
#                cartesian_pose_publisher
#        RUNTIME DESTINATION lib/${PROJECT_NAME})


#############
## Testing ##
#############

#if(BUILD_TESTING)
#find_package(launch_testing_ament_cmake)
#add_launch_test(test/simple_launch.test.py)
#add_launch_test(test/topics.test.py)
#add_launch_test(test/complex_launch.test.py)
#add_launch_test(test/solver_output.test.py)
#install(DIRECTORY test
#        DESTINATION share/${PROJECT_NAME})
#endif()

#############
## Doxygen ##
#############

#find_package(Doxygen)
#if(BUILD_DOC)
#  if(DOXYGEN_FOUND)
#    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
#    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
#    configure_file(${doxyfile_in} ${doxyfile} @ONLY)
#    message("Doxygen build started.")
#    add_custom_target(doc ALL
#                      COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
#                      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#                     COMMENT "Generating API documentation with Doxygen"
#                      VERBATIM)
#   install(DIRECTORY doc/images DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/doc)
#  else(DOXYGEN_FOUND)
#     message("Doxygen needs to be installed to generate the doxygen documentation")
#  endif(DOXYGEN_FOUND)
#endif (BUILD_DOC)
