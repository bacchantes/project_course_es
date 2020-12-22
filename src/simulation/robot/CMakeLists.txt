cmake_minimum_required(VERSION 2.8.3)
project(robot)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
  genmsg
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(

)

add_executable(twist_to_ackermann src/twist_to_ackermann.cpp)

#add_library(${PROJECT_NAME}
#  src/simple_world_plugin.cpp
#  src/gazebo_ros_interface.cpp
#  )
target_link_libraries(twist_to_ackermann ${catkin_LIBRARIES})
