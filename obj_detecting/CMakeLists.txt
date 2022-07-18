cmake_minimum_required(VERSION 2.8.3)
project(obj_detecting)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
  darknet_ros_msgs
  image_transport
  cv_bridge
)

find_package(
  OpenCV REQUIRED
)

include_directories(${OpenCV_INCLUDE_DIRS})

add_message_files(
  FILES
    BoundingBox.msg
    BoundingBoxes.msg
    ObjectCount.msg
)


generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS
  darknet_ros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(obj_detecting src/obj_detecting.cpp)


add_dependencies(obj_detecting ${PROJECT_NAME}_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS})


target_link_libraries(obj_detecting ${catkin_LIBRARIES})
target_link_libraries(obj_detecting ${OpenCV_LIBS})



