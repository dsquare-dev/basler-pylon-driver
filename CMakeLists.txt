cmake_minimum_required(VERSION 3.0.2)
project(basler-capture)

add_definitions("-std=gnu++11")
set(
	CATKIN_COMPONENTS
	 actionlib
	 #camera_control_msgs
	 #camera_info_manager
	 cv_bridge
	 #dianostic_updater
	 image_geometry
	 image_transport
	 roscpp
	 roslaunch
	 sensor_msgs
)

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
	include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS}
  dnb_msgs
  roslint
  #std_msgs
  std_srvs
)

find_package(kiwibot_lib REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
)

set(
	ROSLINT_CPP_OPTS
	 "--extensions=cpp,h,hpp" "--filter=-runtime/references,-readability/todo,-build/include_what_you_use"
)

roslint_cpp(
	src/${PROJECT_NAME}/main.cpp
)

roslaunch_add_file_check(launch)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
	${Pylon_INCLUDE_DIRS}
	${kiwibot_lib_INCLUDE_DIRS}
)

#target_link_libraries(
#	${PROJECT_NAME}
#	 ${catkin_LIBRARIES}
#	 ${Pylon_LIBRARIESS}
#)

#add_dependencies(
#	${PROJECT_NAME}
#	 ${catkin_EXPORTED_TARGETS}
#)

add_executable(
	${PROJECT_NAME}_node
	 src/${PROJECT_NAME}/main.cpp
)

target_link_libraries(
	${PROJECT_NAME}_node
	 ${catkin_LIBRARIES}
	 ${Pylon_LIBRARIES}
	 ${kiwibot_lib_LIBRARIES}
)

add_dependencies(
	${PROJECT_NAME}_node
	 ${catkin_EXPORTED_TARGETS}
)

#catkin_python_setup()

install(
	DIRECTORY
	 launch/
	DESTINATION
	 ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	FILES_MATCHING PATTERN "*.launch"
)

install(
	DIRECTORY
	 config/
	DESTINATION
	 ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
	FILES_MATCHING PATTERN "*.yaml"
)

install(
	DIRECTORY
	 module_config/
	DESTINATION
	 ${CATKIN_PACKAGE_SHARE_DESTINATION}/module_config
	FILES_MATCHING PATTERN "*.yaml"
)

install(
	TARGETS
	 ${PROJECT_NAME}_node
	LIBRARY DESTINATION
	 ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION
	 ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	DIRECTORY
	 include/${PROJECT_NAME}/
	DESTINATION
	 ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN
	 "*.h"
	PATTERN "internal" EXCLUDE
)

file(GLOB children ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(child ${children})
	if(IS_DIRECTORY ${child})
		file(GLOB_RECURSE dir_files "${child}/*")
		list(APPEND ${PROJECT_NAME}_extra_files ${dir_files})
	endif()
endforeach()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${${PROJECT_NAME}_extra_files})





