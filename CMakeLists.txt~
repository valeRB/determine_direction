cmake_minimum_required(VERSION 2.8.3)
project(transform_ft)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs std_srvs)
find_package(Boost REQUIRED system filesystem date_time thread)

## Generate added messages and services
# sgenerate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
 CATKIN_DEPENDS 
 rospy geometry_msgs std_srvs
)

find_package(Eigen REQUIRED)

## Build talker and listener
#include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})



#add_executable(move_both_arms src/move_both_arms.cpp)
#target_link_libraries(move_both_arms ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# install(TARGETS planning_scene_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

