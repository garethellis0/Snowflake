cmake_minimum_required(VERSION 2.8.3)
project(sb_geom)

add_definitions(-std=c++14)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        genmsg
        roscpp
        std_msgs
        message_generation
        )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
        DIRECTORY msg
        FILES Point2D.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES std_msgs
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
        INCLUDE_DIRS include
        LIBRARIES sb_geom
        CATKIN_DEPENDS roscpp message_runtime std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ./include
)

## Declare C++ libraries

# Find all the `.h` and `.cpp` files for the `sb_geom` library
file(GLOB sb_geom_SRCS
        "src/*.cpp"
        )
file(GLOB sb_geom_HEADERS
        "include/*.h"
        )
add_library(sb_geom
        ${sb_geom_SRCS}
        ${sb_geom_HEADERS}
        )


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

    catkin_add_gtest(Point2D-test
            test/Point2D-test.cpp
            )
    target_link_libraries(Point2D-test ${catkin_LIBRARIES})
    # We need to declare this extra dependecy to ensure custom messages
    # in this package are generated first (since this test is in the same
    # package that the messages are being generated in)
    add_dependencies(Point2D-test sb_geom_generate_messages_cpp)

endif()

