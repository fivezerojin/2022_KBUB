cmake_minimum_required(VERSION 3.0.2)
project(control_to_serial)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  rospy
  serial
  std_msgs
)



add_message_files(FILES control2serial.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime roscpp rospy serial std_msgs)








include_directories(${catkin_INCLUDE_DIRS})

add_executable(newSerial src/newSerial.cpp)
add_dependencies(newSerial ${${PROJECT_NAME}_EXPORTED_TARGEETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(newSerial ${catkin_LIBRARIES})

add_executable(Serial src/Serial.cpp)
add_dependencies(Serial ${${PROJECT_NAME}_EXPORTED_TARGEETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(Serial ${catkin_LIBRARIES})

add_executable(Serial_lidar src/Serial_lidar.cpp)
add_dependencies(Serial_lidar ${${PROJECT_NAME}_EXPORTED_TARGEETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(Serial_lidar ${catkin_LIBRARIES})

add_executable(stanley src/stanley.cpp)
add_dependencies(stanley ${${PROJECT_NAME}_EXPORTED_TARGEETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(stanley ${catkin_LIBRARIES})

add_executable(Track src/Track.cpp)
add_dependencies(Track ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Track ${catkin_LIBRARIES})

















