cmake_minimum_required(VERSION 2.8.3)
project(aubo_robot_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    message_generation 
    cv_bridge 
    image_transport 
    tf
    ar_track_alvar_msgs
    apriltag_ros
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
# include_directories("/usr/include/eigen3")
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(FILES
  fetch_box_num.srv
  fetch_marker_id.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
    INCLUDE_DIRS include
    LIBRARIES aubo_robot_base
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        link_directories(${catkin_LIB_DIRS}  lib/lib64)
else()
        link_directories(${catkin_LIB_DIRS}  lib/lib32)
endif()

add_library(robot_control_base src/robot_control_base.cpp)

add_executable(image_draw src/image_draw.cpp)
add_dependencies(image_draw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_draw ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ar_tracker_fetch src/ar_tracker_fetch.cpp src/robot_control_base.cpp)
target_link_libraries(ar_tracker_fetch       
    libauborobotcontroller.so
    libotgLib.a
    robot_control_base
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
)

add_executable(aruco_fetch src/aruco_fetch.cpp src/robot_control_base.cpp)
target_link_libraries(aruco_fetch       
    libauborobotcontroller.so
    libotgLib.a
    robot_control_base
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
)

add_executable(apriltag_fetch src/apriltag_fetch.cpp src/robot_control_base.cpp)
target_link_libraries(apriltag_fetch       
    libauborobotcontroller.so
    libotgLib.a
    robot_control_base
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
)

add_executable(demo11 src/demo11.cpp src/robot_control_base.cpp)
target_link_libraries(demo11       
    libauborobotcontroller.so
    libotgLib.a
    robot_control_base
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/aubo_robot_base.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/aubo_robot_base_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_aubo_robot_base.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
