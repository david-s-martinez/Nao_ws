cmake_minimum_required(VERSION 2.8.3)
project(nao_control_tutorial_3)


find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  naoqi_bridge_msgs
  std_srvs
  tf
  actionlib_msgs
  geometry_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nao_3 src/main.cpp)

target_link_libraries(nao_3 ${catkin_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(nao_3 ${Boost_LIBRARIES})
