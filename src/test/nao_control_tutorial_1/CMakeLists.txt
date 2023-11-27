cmake_minimum_required(VERSION 2.8.3)
project(nao_control_tutorial_1)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
)

add_service_files(
    DIRECTORY srv
    FILES
    MoveJoints.srv
    InterpolationJoints.srv
    TrackAruco.srv
    InterpolationMultiJoints.srv
    CartesianPositionOrientation.srv

)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ./script/move_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(nao_1 src/client.cpp)
target_link_libraries(nao_1 ${catkin_LIBRARIES})
