cmake_minimum_required(VERSION 2.8.3)
project(mapping_urc)

# Build in "Release" (with lots of compiler optimizations) by default
# (If built in "Debug", some functions can take orders of magnitude longer)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

# This exports the compile commands so people using Vim with YouCompleteMe
# can have easy autocompletion
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(--std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sb_utils
    sb_geom
    mapping_msgs_urc
)

# Bring in the `multi_resolution_graph` library
add_subdirectory(multi_resolution_graph)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
    ./include
)

## Declare a C++ executable
add_executable(mapper_node 
    src/mapper_node.cpp
    src/MapperNode.cpp
        src/Mapper.cpp include/Mapper.h src/WorldMap.cpp)
target_link_libraries(mapper_node ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
# TODO: Issue #348

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(WorldMap-test test/WorldMap-test.cpp src/WorldMap.cpp)
    target_link_libraries(WorldMap-test ${catkin_LIBRARIES})

    #TODO: rosTest (Issue #348)

endif()

