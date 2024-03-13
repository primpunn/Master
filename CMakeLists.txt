cmake_minimum_required(VERSION 3.1.3)
project(Master)

#find_package(catkin REQUIRED) #This is original
find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	rospy
	std_msgs
	geometry_msgs
	urdf
	xacro
	message_generation
	)

#catkin_package() #This is original
catkin_package(
	CATKIN_DEPENDS
	geometry_msgs
	roscpp
	rospy
	std_msgs
	)

#Below this is also original
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
