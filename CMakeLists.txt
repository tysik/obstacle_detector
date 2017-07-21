cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detector)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp roslaunch nodelet rviz std_msgs std_srvs geometry_msgs sensor_msgs tf laser_geometry message_generation)
find_package(Armadillo REQUIRED)
find_package(Boost 1.54.0 REQUIRED system)

add_message_files(FILES CircleObstacle.msg SegmentObstacle.msg Obstacles.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scans_merger obstacle_extractor obstacle_tracker obstacle_publisher ${PROJECT_NAME}_nodelets ${PROJECT_NAME}_gui
  CATKIN_DEPENDS roscpp nodelet rviz std_msgs std_srvs geometry_msgs sensor_msgs tf laser_geometry message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#
# Build libs
#
add_library(scans_merger src/scans_merger.cpp)
target_link_libraries(scans_merger ${catkin_LIBRARIES})
add_dependencies(scans_merger ${catkin_EXPORTED_TARGETS})

add_library(obstacle_extractor src/obstacle_extractor.cpp)
target_link_libraries(obstacle_extractor ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(obstacle_extractor ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(obstacle_tracker src/obstacle_tracker.cpp)
target_link_libraries(obstacle_tracker ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(obstacle_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(obstacle_publisher src/obstacle_publisher.cpp)
target_link_libraries(obstacle_publisher ${catkin_LIBRARIES})
add_dependencies(obstacle_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#
# Build nodes
#
add_executable(scans_merger_node src/nodes/scans_merger_node.cpp)
target_link_libraries(scans_merger_node scans_merger)

add_executable(obstacle_extractor_node src/nodes/obstacle_extractor_node.cpp)
target_link_libraries(obstacle_extractor_node obstacle_extractor)

add_executable(obstacle_tracker_node src/nodes/obstacle_tracker_node.cpp)
target_link_libraries(obstacle_tracker_node obstacle_tracker)

add_executable(obstacle_publisher_node src/nodes/obstacle_publisher_node.cpp)
target_link_libraries(obstacle_publisher_node obstacle_publisher)

#
# Build nodelets
#
add_library(${PROJECT_NAME}_nodelets
  src/nodelets/scans_merger_nodelet.cpp
  src/nodelets/obstacle_extractor_nodelet.cpp
  src/nodelets/obstacle_tracker_nodelet.cpp
  src/nodelets/obstacle_publisher_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets scans_merger obstacle_extractor obstacle_tracker obstacle_publisher)

#
# Build rviz plugins
#
set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}_gui
  src/displays/obstacles_display.cpp include/${PROJECT_NAME}/displays/obstacles_display.h
  src/displays/circle_visual.cpp include/${PROJECT_NAME}/displays/circle_visual.h
  src/displays/segment_visual.cpp include/${PROJECT_NAME}/displays/segment_visual.h
  #
  src/panels/scans_merger_panel.cpp include/${PROJECT_NAME}/panels/scans_merger_panel.h
  src/panels/obstacle_extractor_panel.cpp include/${PROJECT_NAME}/panels/obstacle_extractor_panel.h
  src/panels/obstacle_tracker_panel.cpp include/${PROJECT_NAME}/panels/obstacle_tracker_panel.h
  src/panels/obstacle_publisher_panel.cpp include/${PROJECT_NAME}/panels/obstacle_publisher_panel.h)
target_link_libraries(${PROJECT_NAME}_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_gui ${catkin_EXPORTED_TARGETS})

#
# Install libraries
#
install(TARGETS scans_merger obstacle_extractor obstacle_tracker obstacle_publisher ${PROJECT_NAME}_nodelets ${PROJECT_NAME}_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#
# Install nodes
#
install(TARGETS scans_merger_node obstacle_extractor_node obstacle_tracker_node obstacle_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#
# Install header files
#
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#
# Install nodelet and rviz plugins description
#
install(FILES nodelet_plugins.xml rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#
# Install launch files
#
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#
# Create folders and copy resources
#
file(MAKE_DIRECTORY $ENV{HOME}/.local/share/icons/robor)
file(COPY resources/play.png resources/stop.png DESTINATION $ENV{HOME}/.local/share/icons/robor)
