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
)

link_directories(
  ${wbc-core_LIBRARIES}
  ${wbc-controllers_LIBRARIES}
  ${wbc-types_LIBRARIES}
  ${wbc-tasks_LIBRARIES}
)

generate_parameter_library(whole_body_controller_parameters
  src/whole_body_controller_parameters.yaml
)
generate_parameter_library(cartesian_position_controller_parameters
  src/cartesian_position_controller_parameters.yaml
)
generate_parameter_library(joint_position_controller_parameters
  src/joint_position_controller_parameters.yaml
)
generate_parameter_library(com_position_controller_parameters
  src/com_position_controller_parameters.yaml
)

add_library(whole_body_controller SHARED src/whole_body_controller.cpp src/conversions.cpp src/task_interface.cpp)
add_library(cartesian_position_controller SHARED src/cartesian_position_controller.cpp src/conversions.cpp)

ament_target_dependencies(whole_body_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(cartesian_position_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(whole_body_controller PUBLIC dl ${wbc-core_LIBRARIES} ${wbc-types_LIBRARIES} ${wbc-tasks_LIBRARIES} whole_body_controller_parameters)
target_compile_features(whole_body_controller PUBLIC cxx_std_17)
target_include_directories(whole_body_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/wbc_ros>
)
target_link_libraries(cartesian_position_controller PUBLIC dl ${wbc-core_LIBRARIES} ${wbc-types_LIBRARIES} ${wbc-controllers_LIBRARIES} cartesian_position_controller_parameters)
target_compile_features(cartesian_position_controller PUBLIC cxx_std_17)
target_include_directories(cartesian_position_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/wbc_ros>
)

rclcpp_components_register_node(whole_body_controller PLUGIN "wbc_ros::WholeBodyController" EXECUTABLE whole_body_controller_node)
rclcpp_components_register_node(cartesian_position_controller PLUGIN "wbc_ros::CartesianPositionController" EXECUTABLE cartesian_position_controller_node)

install(
  DIRECTORY include/
  DESTINATION include/wbc_ros
)
install(TARGETS whole_body_controller
                cartesian_position_controller
                whole_body_controller_parameters
                cartesian_position_controller_parameters
                joint_position_controller_parameters
                com_position_controller_parameters
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
ament_target_dependencies(joint_trajectory_publisher rclcpp trajectory_msgs)
ament_target_dependencies(cartesian_trajectory_publisher rclcpp wbc_msgs)
ament_target_dependencies(cartesian_pose_publisher rclcpp wbc_msgs)
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
