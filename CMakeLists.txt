cmake_minimum_required(VERSION 2.8.3)
project(robobo)

set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros gazebo_plugins robobo_msgs rospy)

## System dependencies
find_package(GAZEBO REQUIRED)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS
	include

	LIBRARIES
	left_wheel_motor
	right_wheel_motor
	pan_motor
	tilt_motor
	encoders

	CATKIN_DEPENDS
	roscpp
	rospy
	gazebo_ros
	gazebo_plugins

	DEPENDS
	GAZEBO
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# Next line is necessary because Gazebo plugins are not added to GAZEBO_LIBRARIES
link_directories(${GAZEBO_LIBRARY_DIRS})

# Left Motor for individual movement
add_library(left_wheel_motor SHARED plugins/left_wheel_motor.cpp)
add_dependencies(left_wheel_motor ${catkin_EXPORTED_TARGETS})
target_link_libraries(left_wheel_motor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Right Motor for individual movement
add_library(right_wheel_motor SHARED plugins/right_wheel_motor.cpp)
add_dependencies(right_wheel_motor ${catkin_EXPORTED_TARGETS})
target_link_libraries(right_wheel_motor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Pan Motor for individual movement
add_library(pan_motor SHARED plugins/pan_motor.cpp)
add_dependencies(pan_motor ${catkin_EXPORTED_TARGETS})
target_link_libraries(pan_motor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Tilt Motor for individual movement
add_library(tilt_motor SHARED plugins/tilt_motor.cpp)
add_dependencies(tilt_motor ${catkin_EXPORTED_TARGETS})
target_link_libraries(tilt_motor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Motor encoders to know joints position
add_library(encoders SHARED plugins/encoders.cpp)
add_dependencies(encoders ${catkin_EXPORTED_TARGETS})
target_link_libraries(encoders ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Plugin to read infrared sensors
#add_library(pruebas SHARED plugins/pruebas.cpp)
#add_dependencies(pruebas ${catkin_EXPORTED_TARGETS})
#target_link_libraries(pruebas ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Plugin to read infrared sensors
add_library(infrared_range SHARED plugins/infrared_range.cpp)
add_dependencies(infrared_range ${catkin_EXPORTED_TARGETS})
target_link_libraries(infrared_range ${catkin_LIBRARIES} ${Boost_LIBRARIES} RayPlugin)



#############
## Install ##
#############
catkin_install_python(PROGRAMS
  nodes/robobo_services_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
