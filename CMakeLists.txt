cmake_minimum_required(VERSION 2.8.3)
project(youbot_vs)

set(EXTRA_CXX_FLAGS "-std=c++11 -I /home/alisa/visp_ws/librealsense/include/librealsense2")
add_compile_options(-std=c++11 -I /usr/include/eigen3/)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	sensor_msgs
	message_generation
	brics_actuator
	tf
	message_filters
	resource_retriever
	youbot_arm_kinematics
)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

find_package(Boost REQUIRED 
	COMPONENTS filesystem system signals regex date_time program_options thread
)

include_directories(
	${VISP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  /home/alisa/visp_ws/librealsense/include/librealsense2
)

link_libraries(${VISP_LIBRARIES})

catkin_package(
   CATKIN_DEPENDS
	   geometry_msgs
	   message_filters
	   roscpp
	   sensor_msgs
	   std_msgs
	   youbot_arm_kinematics
	   message_runtime
)

add_executable(youbot_vs_node src/youbot_vs_node.cpp)
target_link_libraries(youbot_vs_node  ${catkin_LIBRARIES} ${VISP_LIBRARIES})
add_dependencies(youbot_vs_node ${catkin_EXPORTED_TARGETS})
