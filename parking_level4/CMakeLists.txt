cmake_minimum_required(VERSION 2.8.3)
project(parking_level4)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  message_generation

)
find_package(OpenCV REQUIRED)

add_message_files(FILES stopmsg.msg)

generate_messages(DEPENDENCIES std_msgs)

include_directories(${OpenCV_INCLUDE_DIRS})


catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})


add_executable(parking_level4 src/parking_level4.cpp)
target_link_libraries(parking_level4 ${catkin_LIBRARIES})
target_link_libraries(parking_level4 ${OpenCV_LIBS})

