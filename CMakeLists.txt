cmake_minimum_required(VERSION 3.0.2)
project(wbc_ros)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  wbc_msgs
  roscpp
  message_generation
)

include(FindPkgConfig)
pkg_check_modules(wbc-core REQUIRED wbc-core)
pkg_check_modules(wbc-scenes REQUIRED wbc-scenes)
pkg_check_modules(wbc-controllers REQUIRED wbc-controllers)
pkg_check_modules(wbc-robot_models-pinocchio REQUIRED wbc-robot_models-pinocchio)
pkg_check_modules(wbc-solvers-hls REQUIRED wbc-solvers-hls)
pkg_check_modules(wbc-solvers-qpoases REQUIRED wbc-solvers-qpoases)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#    FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs trajectory_msgs wbc_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${wbc-core_INCLUDE_DIRS}
  ${wbc-scenes_INCLUDE_DIRS}
  ${wbc-controllers_INCLUDE_DIRS}
  ${wbc-robot_models-kdl_INCLUDE_DIRS}
  ${wbc-solvers-hls_INCLUDE_DIRS}
  ${wbc-solvers-qpoases_INCLUDE_DIRS}
)

link_directories(
  ${wbc-core_LIBRARIES}
  ${wbc-scenes_LIBRARIES}
  ${wbc-controllers_LIBRARIES}
  ${wbc-robot_models-kdl_LIBRARIES}
  ${wbc-solvers-hls_LIBRARIES}
  ${wbc-solvers-qpoases_LIBRARIES}
)

add_executable(wbc_node nodes/wbc_node.cpp nodes/conversions.cpp)
add_executable(loop_back_driver_node nodes/loop_back_driver_node.cpp)
add_executable(cartesian_position_controller_node nodes/controllers/cartesian_position_controller_node.cpp nodes/controllers/controller_node.cpp nodes/conversions.cpp)
add_executable(joint_position_controller_node nodes/controllers/joint_position_controller_node.cpp  nodes/controllers/controller_node.cpp nodes/conversions.cpp)
add_executable(cartesian_force_controller_node nodes/controllers/cartesian_force_controller_node.cpp  nodes/controllers/controller_node.cpp nodes/conversions.cpp)
add_executable(cartesian_radial_potential_fields_node nodes/controllers/cartesian_radial_potential_fields_node.cpp  nodes/controllers/controller_node.cpp nodes/conversions.cpp)
add_executable(joint_limit_avoidance_node nodes/controllers/joint_limit_avoidance_node.cpp  nodes/controllers/controller_node.cpp nodes/conversions.cpp)

target_link_libraries(wbc_node dl
   ${catkin_LIBRARIES}
   ${roscpp_LIBRARIES}
   ${wbc-core_LIBRARIES}
   ${wbc-scenes_LIBRARIES}
   ${wbc-robot_models-kdl_LIBRARIES}
   ${wbc-solvers-hls_LIBRARIES}
   ${wbc-solvers-qpoases_LIBRARIES}
)
target_link_libraries(loop_back_driver_node
   ${roscpp_LIBRARIES}
)
target_link_libraries(cartesian_position_controller_node
   ${roscpp_LIBRARIES}
   ${wbc-controllers_LIBRARIES}
)
target_link_libraries(joint_position_controller_node
   ${roscpp_LIBRARIES}
   ${wbc-controllers_LIBRARIES}
)
target_link_libraries(cartesian_force_controller_node
    ${roscpp_LIBRARIES}
    ${wbc-controllers_LIBRARIES}
)
target_link_libraries(cartesian_radial_potential_fields_node
    ${roscpp_LIBRARIES}
    ${wbc-controllers_LIBRARIES}
)
target_link_libraries(joint_limit_avoidance_node
    ${roscpp_LIBRARIES}
    ${wbc-controllers_LIBRARIES}
)


install(TARGETS wbc_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
