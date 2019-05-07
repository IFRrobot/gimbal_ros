cmake_minimum_required(VERSION 2.8.3)
project(gimbal_node)

IF(CMAKE_COMPILER_IS_GNUCC)
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)



add_message_files(
  FILES
  gimbal.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

add_executable(gimbal_node
        src/uart.cpp
        include/uart.h
        src/gimbal_main_node.cpp 
        include/main.h
        src/protocol.cpp
        include/protocol.h
        src/handle.cpp
        include/handle.h
       )
target_link_libraries(gimbal_node ${catkin_LIBRARIES})
add_dependencies(gimbal_node gimbal_node_generate_message_cpp)

add_executable(gimbal_teleop_node
               src/gimbal_teleop.cpp       
       )
target_link_libraries(gimbal_teleop_node ${catkin_LIBRARIES})
add_dependencies(gimbal_teleop_node gimbal_node_generate_message_cpp)
