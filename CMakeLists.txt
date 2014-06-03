cmake_minimum_required(VERSION 2.8.3)
project(leap_client)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -O3 -DNDEBUG")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs genmsg tf visualization_msgs)

# PATH TO LEAPSDK
set(LEAPSDK $ENV{LEAPSDK})
if(NOT LEAPSDK)
    message(FATAL ERROR " The LEAPSDK environment variable is not set")
endif()

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES HandInfo.msg FingerInfo.msg HandInfoList.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs leap_client) 

catkin_package(INCLUDE_DIRS include DEPENDS message_runtime std_msgs geometry_msgs tf visualization_msgs)

###########
## Build ##
###########
include_directories(include ${INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} ${LEAPSDK}/include)
if(CMAKE_SIZEOF_VOID_P EQUAL 8) 
    set(arch_64 TRUE) 
else() 
    set(arch_64 FALSE) 
endif()

if(arch_64)
    link_directories(${LINK_DIRECTORIES} ${LEAPSDK}/lib/x64)
else()
    link_directories(${LINK_DIRECTORIES} ${LEAPSDK}/lib/x86)
endif()

## Declare a cpp library
add_library(leap_ros_listener src/LeapROSListener.cpp)
target_link_libraries(leap_ros_listener Leap ${catkin_LIBRARIES})
add_dependencies(leap_ros_listener leap_client_gencpp)

## Declare a cpp executable
add_executable(leap_client_node src/leap_client_node.cpp)
target_link_libraries(leap_client_node ${catkin_LIBRARIES} leap_ros_listener)
add_dependencies(leap_client_node leap_client_gencpp)

#add_executable(static_twist_controller src/static_twist_controller.cpp)
#target_link_libraries(static_twist_controller ${catkin_LIBRARIES})
#add_dependencies(static_twist_controller leap_client_gencpp)

add_executable(leap_viz src/leap_viz.cpp)
target_link_libraries(leap_viz ${catkin_LIBRARIES})
add_dependencies(leap_viz leap_client_gencpp)
