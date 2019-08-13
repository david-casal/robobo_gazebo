cmake_minimum_required(VERSION 2.8.3)
project(robobo_gazebo)

set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	gazebo_ros 
	gazebo_plugins 
	robobo_msgs 
	rospy 
)


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
	move_wheels
	move_pan_tilt
	encoders
	infrared_range
	robobo_camera_utils
	robobo_camera
	robobo_imu_sensor

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


# Plugin for MoveWheels service
add_library(move_wheels SHARED plugins/move_wheels.cpp)
add_dependencies(move_wheels ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_wheels ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Plugin for MovePanTilt service
add_library(move_pan_tilt SHARED plugins/move_pan_tilt.cpp)
add_dependencies(move_pan_tilt ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_pan_tilt ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Motor encoders to know joints position and ResetWheels service
add_library(encoders SHARED plugins/encoders.cpp)
add_dependencies(encoders ${catkin_EXPORTED_TARGETS})
target_link_libraries(encoders ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Plugin to read infrared sensors
add_library(infrared_range SHARED plugins/infrared_range.cpp)
add_dependencies(infrared_range ${catkin_EXPORTED_TARGETS})
target_link_libraries(infrared_range ${catkin_LIBRARIES} ${Boost_LIBRARIES} RayPlugin)

# Camera utils plugin
add_library(robobo_camera_utils SHARED plugins/robobo_camera_utils.cpp)
add_dependencies(robobo_camera_utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(robobo_camera_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES} CameraPlugin)

# Camera plugin
add_library(robobo_camera SHARED plugins/robobo_camera.cpp)
add_dependencies(robobo_camera ${catkin_EXPORTED_TARGETS})
target_link_libraries(robobo_camera ${catkin_LIBRARIES} ${Boost_LIBRARIES} CameraPlugin)

# Imu sensor plugin
add_library(robobo_imu_sensor SHARED plugins/robobo_imu_sensor.cpp)
add_dependencies(robobo_imu_sensor ${catkin_EXPORTED_TARGETS})
target_link_libraries(robobo_imu_sensor ${catkin_LIBRARIES})



#############
## Install ##
#############
catkin_install_python(
	PROGRAMS
	nodes/robobo_irs

	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	TARGETS
	move_wheels
	move_pan_tilt
	encoders
	infrared_range
	robobo_camera_utils
	robobo_camera
	robobo_imu_sensor

	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
	DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  	DIRECTORY models/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(
	DIRECTORY worlds/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
#############
## Testing ##
#############
