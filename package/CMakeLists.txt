cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_motion_with_platform_control)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)


# Directories definition
set(QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_SOURCE_DIR
  src/source
)

set(QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_INCLUDE_DIR
  src/include
)

set(QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_HEADER_FILES
        ${QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_INCLUDE_DIR}/behavior_move_at_speed_with_platform_control.h 
        ${QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_INCLUDE_DIR}/behavior_rotate_with_platform_control.h
        ${QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_INCLUDE_DIR}/behavior_move_vertical_with_platform_control.h
)

set(QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_SOURCE_FILES
        ${QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_SOURCE_DIR}/behavior_move_at_speed_with_platform_control.cpp 
        ${QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_SOURCE_DIR}/behavior_rotate_with_platform_control.cpp
        ${QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_SOURCE_DIR}/behavior_move_vertical_with_platform_control.cpp
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  behavior_execution_manager
  behavior_execution_manager_msgs
  geometry_msgs
  aerostack_msgs
)

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
  INCLUDE_DIRS ${QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  std_msgs
  behavior_execution_manager
  behavior_execution_manager_msgs
  geometry_msgs
  aerostack_msgs
  DEPENDS yaml-cpp
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
add_executable(BehaviorMoveAtSpeedWithPlatformControl ${QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_SOURCE_DIR}/behavior_move_at_speed_with_platform_control.cpp)
target_link_libraries(BehaviorMoveAtSpeedWithPlatformControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorMoveAtSpeedWithPlatformControl yaml-cpp)
add_executable(BehaviorRotateWithPlatformControl ${QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_SOURCE_DIR}/behavior_rotate_with_platform_control.cpp)
target_link_libraries(BehaviorRotateWithPlatformControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorRotateWithPlatformControl yaml-cpp)
add_executable(BehaviorMoveVerticalWithPlatformControl ${QUADROTOR_MOTION_WITH_PLATFORM_CONTROL_SOURCE_DIR}/behavior_move_vertical_with_platform_control.cpp)
target_link_libraries(BehaviorMoveVerticalWithPlatformControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorMoveVerticalWithPlatformControl yaml-cpp)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(BehaviorMoveAtSpeedWithPlatformControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorRotateWithPlatformControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorMoveVerticalWithPlatformControl ${catkin_EXPORTED_LIBRARIES})
endif()
