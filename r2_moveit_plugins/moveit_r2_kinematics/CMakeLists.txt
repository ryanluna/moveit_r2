cmake_minimum_required(VERSION 2.8.3)
project(moveit_r2_kinematics)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  nasa_robodyn
)

catkin_package(
  LIBRARIES moveit_r2_tree_kinematics moveit_r2_chain_kinematics
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    nasa_robodyn
)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_library(moveit_r2_tree_kinematics src/moveit_r2_tree_kinematics.cpp)
target_link_libraries(moveit_r2_tree_kinematics ${catkin_LIBRARIES})

add_library(moveit_r2_chain_kinematics src/moveit_r2_chain_kinematics.cpp)
target_link_libraries(moveit_r2_chain_kinematics ${catkin_LIBRARIES})

install(TARGETS moveit_r2_tree_kinematics moveit_r2_chain_kinematics LIBRARY DESTINATION lib)
install(FILES moveit_r2_chain_kinematics_plugin_description.xml moveit_r2_tree_kinematics_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
