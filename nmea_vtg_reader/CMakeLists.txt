cmake_minimum_required(VERSION 3.0.2)
project(nmea_vtg_reader)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  geometry_msgs
  std_msgs
  nmea_msgs
  nav_msgs
  message_generation
  eigen_conversions
  
)





## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   vel_msg.msg
   gps_heading.msg
   
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  EXPORTED_TARGETS nmea_vtg_reader_generate_messages_cpp
  INCLUDE_DIRS include
  LIBRARIES nmea_vtg_reader
  CATKIN_DEPENDS roscpp serial geometry_msgs std_msgs nmea_msgs nav_msgs message_runtime
  DEPENDS ${EIGEN_PACKAGE}

)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})


add_executable(nmea_vtg_reader src/VTG_reader.cpp)
add_executable(heading_using_vector src/heading_using_vector.cpp)

#add_dependencies(nmea_vtg_reader nmea_vtg_reader_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS})
target_link_libraries(nmea_vtg_reader ${catkin_LIBRARIES})
target_link_libraries(heading_using_vector ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(heading_using_vector nmea_vtg_reader_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   src/kalman_orientation_reader.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



