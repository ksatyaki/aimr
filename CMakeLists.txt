cmake_minimum_required(VERSION 2.8.3)
project(aimr)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  simple_service
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aimr fuzzy fuzzy_velocity
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(fuzzy
  src/fuzzy.cpp
)

add_library(fuzzy_velocity
	src/fuzzy_velocity.cpp
)	

add_executable(way_points_follower src/fuzzy_waypoints_follower.cpp)
add_executable(move_to_fuzzy_server src/fuzzy_goto.cpp)
add_executable(fuzzy_wall_follow src/fuzzy_wall_follow.cpp)

add_dependencies(move_to_fuzzy_server simple_service_generated_messages_cpp)
add_dependencies(way_points_follower simple_service_generated_messages_cpp)

target_link_libraries(fuzzy
  ${catkin_LIBRARIES}
)

target_link_libraries(fuzzy_velocity
  ${catkin_LIBRARIES}
  fuzzy
)

target_link_libraries(move_to_fuzzy_server
	${catkin_LIBRARIES}
	fuzzy
	fuzzy_velocity
)

target_link_libraries(fuzzy_wall_follow
  ${catkin_LIBRARIES}
  fuzzy
  fuzzy_velocity
)

target_link_libraries(way_points_follower
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS aimr aimr_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_aimr.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
