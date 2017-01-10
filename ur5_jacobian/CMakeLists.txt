cmake_minimum_required(VERSION 2.8.3)
project(ur5_jacobian)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  roscpp
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  tf
  keyboard
  hks_gripper
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
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
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur5_jocobian
#  CATKIN_DEPENDS moveit_core moveit_ros_planning_interface roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(ur5_jocobian
#   src/${PROJECT_NAME}/ur5_jocobian.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(ur5_jocobian ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
 add_executable(ur5_move_group_test src/ur5_move_group_test.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(ur5_move_group_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(ur5_move_group_test
   ${catkin_LIBRARIES}
 )
## Declare a C++ executable
 add_executable(ur5_kinematics_test src/ur5_kinematics_test.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(ur5_kinematics_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(ur5_kinematics_test
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(ur5_jacobian src/ur5_jacobian.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(ur5_jacobian ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(ur5_jacobian
   ${catkin_LIBRARIES}
 )
## Declare a C++ executable
 add_executable(ur5_jacobian2 src/ur5_jacobian2.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(ur5_jacobian2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(ur5_jacobian2
   ${catkin_LIBRARIES}
 )
## Declare a C++ executable
 add_executable(ur5_jacobian3 src/ur5_jacobian3.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(ur5_jacobian3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(ur5_jacobian3
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(ur5_joint_speed src/ur5_joint_speed.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(ur5_joint_speed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(ur5_joint_speed
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(follower_goal_broadcaster src/follower_goal_broadcaster.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(follower_goal_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(follower_goal_broadcaster
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(ur5_jacobian_follower src/ur5_jacobian_follower.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(ur5_jacobian_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(ur5_jacobian_follower
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(follower_with_orientation src/follower_with_orientation.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(follower_with_orientation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(follower_with_orientation
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(follower_with_orientation_qr src/follower_with_orientation_qr.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(follower_with_orientation_qr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(follower_with_orientation_qr
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(follower_with_orientation_mouse src/follower_with_orientation_mouse.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(follower_with_orientation_mouse ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(follower_with_orientation_mouse
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(qrcode_track_goal_broadercaster src/qrcode_track_goal_broadercaster.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(qrcode_track_goal_broadercaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(qrcode_track_goal_broadercaster
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(qrcode_track_goal_broadercaster2 src/qrcode_track_goal_broadercaster2.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(qrcode_track_goal_broadercaster2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(qrcode_track_goal_broadercaster2
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(static_goal_broadcaster src/static_goal_broadcaster.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(static_goal_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(static_goal_broadcaster
   ${catkin_LIBRARIES}
 )


## Declare a C++ executable
 add_executable(qrcode_stable_goal_broadcaster src/qrcode_stable_goal_broadcaster.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(qrcode_stable_goal_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(qrcode_stable_goal_broadcaster
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(follower_with_orientation_fetch src/follower_with_orientation_fetch.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(follower_with_orientation_fetch ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(follower_with_orientation_fetch
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(yaskawa_demo src/yaskawa_demo.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(yaskawa_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(yaskawa_demo
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
# install(TARGETS ur5_jocobian ur5_jocobian_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur5_jocobian.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
