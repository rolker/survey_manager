cmake_minimum_required(VERSION 2.8.3)
project(survey_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kongsberg_em_control
  std_msgs
  project11
  roscpp
  nodelet
)

find_library(GEOS_LIBRARY
  NAMES geos
  PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local
    /usr
    /sw
    /opt/local
    /opt/csw
    /opt
    /usr/freeware    
  PATH_SUFFIXES lib64 lib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES SonarCoverage
  CATKIN_DEPENDS std_msgs sensor_msgs project11 roscpp nodelet
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(SonarCoverage_nodelet src/SonarCoverage.cpp)

target_link_libraries(SonarCoverage_nodelet
   ${catkin_LIBRARIES} ${GEOS_LIBRARY}
)

add_executable(${PROJECT_NAME}_node src/survey_manager_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node SonarCoverage_nodelet
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
