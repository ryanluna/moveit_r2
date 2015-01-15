cmake_minimum_required(VERSION 2.8.3)
project(r2_controller_manager)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin COMPONENTS
  roscpp
  pluginlib
  actionlib_msgs
  trajectory_msgs
  nasa_robodyn
  r2_translation_msgs
  moveit_core
  REQUIRED)

# catkin_package defines the env vars used in install later
catkin_package(
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp pluginlib actionlib_msgs trajectory_msgs nasa_robodyn r2_translation_msgs moveit_core
   DEPENDS
   INCLUDE_DIRS include
)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include)

link_directories(${catkin_LIBRARY_DIRS})
add_library(${PROJECT_NAME} src/R2MoveitControllerManager.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  FILES r2_moveit_controller_manager_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
