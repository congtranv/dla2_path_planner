cmake_minimum_required(VERSION 2.8)
project(dla2_path_planner)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE false)

#------------- PARAMETERS ------------------------------------
set(SOME_NON_ROS_LIBRARY_DIR $ENV{SOME_NON_ROS_LIBRARY_DIR})

#-------------CATKIN build system ---------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mav_msgs
  mav_planning_msgs
  visualization_msgs
  tf2
  tf2_ros
  nodelet
)

IF(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_ros
    CATKIN_DEPENDS roscpp std_msgs tf2 tf2_ros nodelet mav_msgs mav_planning_msgs visualization_msgs
  )
ENDIF(catkin_FOUND)

SET( EXTRA_LIBS
    ${EXTRA_LIBS}
    ${catkin_LIBRARIES}
)

find_package(cmake_modules REQUIRED)
#find_package(Eigen) 
find_package(Eigen3) 

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${EIGEN_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

#-------------CATKIN end-------------------------------------
#ADD_DEFINITIONS("-std=c++11")
#ADD_DEFINITIONS("-std=c++0x")

#-----------------------------------------------------------------------------

# ompl
find_package(ompl REQUIRED)
#   checking ompl cmake variables while running: "catkin build"
# message(WARNING "OMPL_INCLUDE_DIRS: " ${OMPL_INCLUDE_DIRS})
# message(WARNING "OMPL_LIBRARY_DIR: " ${OMPL_LIBRARY_DIR})
# message(WARNING "OMPL_LIBRARIES: " ${OMPL_LIBRARIES})
include_directories( ${OMPL_INCLUDE_DIRS} )
link_directories( ${OMPL_LIBRARY_DIR} )
set(EXTRA_LIBS
  ${EXTRA_LIBS}
  ${OMPL_LIBRARIES}
)

# some_non_ros_library
#SET(EXTRA_LIBS
#  ${EXTRA_LIBS}
#  # some_non_ros_library
#)
#include_directories( ${SOME_NON_ROS_LIBRARY_DIR}/src )
#link_directories( ${SOME_NON_ROS_LIBRARY_DIR}/build )
#link_directories(/usr/local/lib)
#set(EXTRA_LIBS
#  ${EXTRA_LIBS}
#  ${SOME_NON_ROS_LIBRARIES}
#)

# --- OMPL_Example_OptimalPlanning ---
add_executable(${PROJECT_NAME}_OMPL_Example_OptimalPlanning src/OMPL_Example_OptimalPlanning.cpp )
target_link_libraries(${PROJECT_NAME}_OMPL_Example_OptimalPlanning
    ${EXTRA_LIBS}
)
# ------------------------------------

# --- DLA2 Path Planner Library ---
SET( SRC
  src/${PROJECT_NAME}/${PROJECT_NAME}_ros.cpp
)

SET( HDR
  include/${PROJECT_NAME}/${PROJECT_NAME}_ros.h
  include/${PROJECT_NAME}/helper_functions.h
)

add_library(${PROJECT_NAME}_ros SHARED ${SRC} ${HDR})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_ros
    ${EXTRA_LIBS}
)
# -----------------------------

# --- DLA2 Path Planner Node / Executable ---
set (EXTRA_LIBS
    ${EXTRA_LIBS}
    ${PROJECT_NAME}_ros
)

##
add_executable(${PROJECT_NAME}_ros_node src/main_ros_node.cpp )
target_link_libraries(${PROJECT_NAME}_ros_node
    ${EXTRA_LIBS}
)
# -----------------------------

# --- trajectory visualization --- 
set(${PROJECT_NAME}_trajectory_visualization_ROS_NODE_SOURCES
		include/trajectory_visualization/trajectory_visualization.h
		src/trajectory_visualization/trajectory_visualization.cpp
		src/trajectory_visualization_ros_node.cpp
)
add_executable(${PROJECT_NAME}_trajectory_visualization ${dla2_path_planner_trajectory_visualization_ROS_NODE_SOURCES})
target_link_libraries(${PROJECT_NAME}_trajectory_visualization
		${catkin_LIBRARIES}
)
# --------------------------------

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_ros_node ${PROJECT_NAME}_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES launch/${PROJECT_NAME}_ros_node.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
