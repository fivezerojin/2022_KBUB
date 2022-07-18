cmake_minimum_required(VERSION 3.0.2)
project(stop_line)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  cv_bridge
  roscpp
)

find_package (OpenCV REQUIRED)


add_message_files(FILES stopmsg.msg)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lane_test
  CATKIN_DEPENDS std_msgs cv_bridge roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS}
)


add_executable(stop src/stop.cpp)
add_dependencies(stop ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(stop ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


