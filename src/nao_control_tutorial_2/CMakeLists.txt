cmake_minimum_required(VERSION 2.8.3)
project(nao_control_tutorial_2)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)

add_service_files(
    DIRECTORY srv
    FILES
    MoveJoints.srv
    CartesianPositionOrientation.srv
    TrackAruco.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ./script/move_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
add_executable(nao_2 src/client.cpp)
target_link_libraries(nao_2 ${catkin_LIBRARIES})

find_package(OpenCV REQUIRED)
target_link_libraries(nao_2 ${OpenCV_LIBS})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake)
target_link_libraries(nao_2  ${aruco_LIBS})
