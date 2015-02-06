cmake_minimum_required(VERSION 2.8.3)
project(baxter_pick_place)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  #moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  moveit_visual_tools
  moveit_grasps
  #baxter_core_msgs
  #std_msgs
  #geometry_msgs
  baxter_control
  roscpp
  cmake_modules
)

#find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
)

# Enable gdb to show line numbers
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
#set(CMAKE_BUILD_TYPE Debug)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Main executable
add_executable(block_pick_place src/block_pick_place.cpp)
target_link_libraries(block_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

