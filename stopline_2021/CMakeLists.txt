cmake_minimum_required(VERSION 2.8.3)
project(stopline_2021)

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


add_executable(stopline_2021 src/stopline_2021.cpp)
target_link_libraries(stopline_2021 ${catkin_LIBRARIES})
target_link_libraries(stopline_2021 ${OpenCV_LIBS})

