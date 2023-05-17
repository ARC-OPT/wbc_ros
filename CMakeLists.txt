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

include_directories(
  ${wbc-core_INCLUDE_DIRS}
  ${wbc-scenes_INCLUDE_DIRS}
  ${wbc-controllers_INCLUDE_DIRS}
)

link_directories(
  ${wbc-core_LIBRARIES}
  ${wbc-scenes_LIBRARIES}
  ${wbc-controllers_LIBRARIES}
)

add_executable(wbc_node src/wbc_node.cpp src/controllers/controller_node.cpp src/conversions.cpp)
add_executable(loop_back_driver_node src/loop_back_driver_node.cpp)
add_executable(cartesian_position_controller_node src/controllers/cartesian_position_controller_node.cpp src/controllers/controller_node.cpp src/conversions.cpp)
add_executable(joint_position_controller_node src/controllers/joint_position_controller_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)
add_executable(cartesian_force_controller_node src/controllers/cartesian_force_controller_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)
add_executable(cartesian_radial_potential_fields_node src/controllers/cartesian_radial_potential_fields_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)
add_executable(joint_limit_avoidance_node src/controllers/joint_limit_avoidance_node.cpp  src/controllers/controller_node.cpp src/conversions.cpp)

ament_target_dependencies(wbc_node rclcpp std_msgs trajectory_msgs sensor_msgs wbc_msgs)
ament_target_dependencies(loop_back_driver_node rclcpp trajectory_msgs sensor_msgs)
ament_target_dependencies(cartesian_position_controller_node rclcpp wbc_msgs sensor_msgs trajectory_msgs)
ament_target_dependencies(joint_position_controller_node rclcpp wbc_msgs sensor_msgs trajectory_msgs)
ament_target_dependencies(cartesian_force_controller_node rclcpp wbc_msgs sensor_msgs trajectory_msgs geometry_msgs)
ament_target_dependencies(cartesian_radial_potential_fields_node rclcpp wbc_msgs sensor_msgs trajectory_msgs)
ament_target_dependencies(joint_limit_avoidance_node rclcpp wbc_msgs sensor_msgs trajectory_msgs)

target_link_libraries(wbc_node dl
   ${wbc-core_LIBRARIES}
   ${wbc-scenes_LIBRARIES}
)
target_link_libraries(cartesian_position_controller_node
   ${wbc-controllers_LIBRARIES}
)
target_link_libraries(joint_position_controller_node
   ${wbc-controllers_LIBRARIES}
)
target_link_libraries(cartesian_force_controller_node
   ${wbc-controllers_LIBRARIES}
)
target_link_libraries(cartesian_radial_potential_fields_node
    ${wbc-controllers_LIBRARIES}
)
target_link_libraries(joint_limit_avoidance_node
    ${wbc-controllers_LIBRARIES}
)

install(TARGETS wbc_node
                loop_back_driver_node
                cartesian_position_controller_node
                joint_position_controller_node
                cartesian_force_controller_node
                cartesian_radial_potential_fields_node
                joint_limit_avoidance_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY models
        DESTINATION share/${PROJECT_NAME}/
)        

ament_package()

#############
## Testing ##
#############
