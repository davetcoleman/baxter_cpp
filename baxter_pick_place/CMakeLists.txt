cmake_minimum_required(VERSION 2.8.3)
project(baxter_pick_place)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  moveit_visual_tools
  moveit_simple_grasps
  baxter_core_msgs
  std_msgs
  geometry_msgs
  baxter_control
  roscpp
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    moveit_simple_grasps
  LIBRARIES 
    baxter_move_group_interface
)

# Enable gdb to show line numbers
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
set(CMAKE_BUILD_TYPE Debug)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(baxter_move_group_interface src/baxter_move_group_interface.cpp)
target_link_libraries(baxter_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Main executable
add_executable(block_pick_place src/block_pick_place.cpp)
target_link_libraries(block_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Demos
add_executable(random_planning src/demos/random_planning.cpp)
target_link_libraries(random_planning baxter_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Tools
add_executable(get_joint_values src/tools/get_joint_values.cpp)
target_link_libraries(get_joint_values ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Test
#add_executable(moveit_simple_grasps_test src/tests/moveit_simple_grasps_test.cpp)
#target_link_libraries(moveit_simple_grasps_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(end_effector_position src/tools/end_effector_position.cpp)
target_link_libraries(end_effector_position ${catkin_LIBRARIES} ${Boost_LIBRARIES})