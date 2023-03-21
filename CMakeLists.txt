cmake_minimum_required(VERSION 3.8)
project(wbc_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(wbc_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include(FindPkgConfig)
pkg_check_modules(wbc-core REQUIRED wbc-core)
pkg_check_modules(wbc-scenes REQUIRED wbc-scenes)
pkg_check_modules(wbc-controllers REQUIRED wbc-controllers)
pkg_check_modules(wbc-robot_models-pinocchio REQUIRED wbc-robot_models-pinocchio)
pkg_check_modules(wbc-solvers-hls REQUIRED wbc-solvers-hls)
pkg_check_modules(wbc-solvers-qpoases REQUIRED wbc-solvers-qpoases)

include_directories(
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

add_executable(wbc_node src/wbc_node.cpp src/controllers/controller_node.cpp src/conversions.cpp)
add_executable(loop_back_driver_node src/loop_back_driver_node.cpp)
#add_executable(cartesian_position_controller_node src/controllers/cartesian_position_controller_node.cpp src/controllers/controller_node.cpp src/conversions.cpp)
#add_executable(joint_position_controller_node src/controllers/joint_position_controller_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)
#add_executable(cartesian_force_controller_node src/controllers/cartesian_force_controller_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)
#add_executable(cartesian_radial_potential_fields_node src/controllers/cartesian_radial_potential_fields_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)
#add_executable(joint_limit_avoidance_node src/controllers/joint_limit_avoidance_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)

ament_target_dependencies(wbc_node rclcpp std_msgs trajectory_msgs sensor_msgs wbc_msgs)
ament_target_dependencies(loop_back_driver_node rclcpp trajectory_msgs sensor_msgs)

target_link_libraries(wbc_node dl
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

#target_link_libraries(cartesian_position_controller_node
#   ${roscpp_LIBRARIES}
#   ${wbc-controllers_LIBRARIES}
#)
#target_link_libraries(joint_position_controller_node
#   ${roscpp_LIBRARIES}
#   ${wbc-controllers_LIBRARIES}
#)
#target_link_libraries(cartesian_force_controller_node
#    ${roscpp_LIBRARIES}
#    ${wbc-controllers_LIBRARIES}
#)
#target_link_libraries(cartesian_radial_potential_fields_node
#    ${roscpp_LIBRARIES}
#    ${wbc-controllers_LIBRARIES}
#)
#target_link_libraries(joint_limit_avoidance_node
#    ${roscpp_LIBRARIES}
#    ${wbc-controllers_LIBRARIES}
#)

install(TARGETS wbc_node
                loop_back_driver_node
  DESTINATION lib/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/test/config/wbc.yml
              ${CMAKE_CURRENT_SOURCE_DIR}/test/config/joints.yml
  DESTINATION test/config)

ament_package()

#############
## Testing ##
#############

#if(CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    add_rostest_gtest(${PROJECT_NAME}-test test/test_wbc.test test/test_wbc.cpp)
#    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${roscpp_LIBRARIES})
#endif()
