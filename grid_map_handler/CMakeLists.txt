cmake_minimum_required(VERSION 2.8.3)
project(grid_map_handler)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Bullet REQUIRED)
find_package(catkin REQUIRED COMPONENTS
roscpp

grid_map_core
grid_map_cv
grid_map_ros
grid_map_msgs

geometry_msgs

cv_bridge
image_transport
costmap_2d

pcl_conversions
pcl_ros
laser_geometry
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES grid_map_handler
    CATKIN_DEPENDS
    roscpp

    grid_map_core
    grid_map_cv
    grid_map_ros
    grid_map_msgs

    geometry_msgs

    cv_bridge
    image_transport
    costmap_2d

    pcl_conversions
    pcl_ros
    laser_geometry
#  DEPENDS system_lib
)

include_directories(
  include
  ${BULLET_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}}
        ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/grid_map_handler.cpp
   include/grid_map_handler/gmh_definitions.h
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/grid_map_handler/grid_map_handler.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${YAMLCPP_LIBRARIES}
        ${PCL_LIBRARIES}
        ${BULLET_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${YAMLCPP_LIBRARIES}
        ${PCL_LIBRARIES}
        ${BULLET_LIBRARIES}
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
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grid_map_handler.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
