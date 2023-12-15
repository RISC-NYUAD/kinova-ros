cmake_minimum_required(VERSION 3.0.2)
project(kinova_control)


find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  geometry_msgs
  std_msgs
  sensor_msgs
  trajectory_msgs
)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    geometry_msgs
    std_msgs
    sensor_msgs
    trajectory_msgs
)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}        
        /usr/include/eigen3
        )

link_libraries(
        ${GAZEBO_LIBRARIES}
        )

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  src/move_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(kinova_mobile_node src/kinova_mobile.cpp src/libraries/utils.cpp)
target_link_libraries(kinova_mobile_node ${catkin_LIBRARIES})

add_executable(kinova_handle_node src/kinova_handler.cpp src/libraries/utils.cpp)
target_link_libraries(kinova_handle_node ${catkin_LIBRARIES})

add_executable(kinova_ee_track_node src/kinova_ball_track.cpp src/libraries/utils.cpp)
target_link_libraries(kinova_ee_track_node ${catkin_LIBRARIES})

add_library(kinova_gripper_plugin SHARED src/gripper_plugin.cpp)
target_link_libraries(kinova_gripper_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(kinova_ball_plugin SHARED src/ball_plugin_kinova.cpp)
target_link_libraries(kinova_ball_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(mobile_base_plugin SHARED src/mobile_base_ctrl.cpp)
target_link_libraries(mobile_base_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

