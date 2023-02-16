cmake_minimum_required(VERSION 2.8.3)
project(survey_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  project11_msgs
  project11
  roscpp
  nodelet
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES SonarCoverage
  CATKIN_DEPENDS std_msgs sensor_msgs project11 roscpp nodelet project11_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(SonarCoverage_nodelet src/SonarCoverage.cpp)
add_dependencies(SonarCoverage_nodelet project11_msgs_gencpp)

target_link_libraries(SonarCoverage_nodelet
   ${catkin_LIBRARIES} 
)

add_executable(${PROJECT_NAME}_node src/survey_manager_node.cpp)
add_dependencies(${PROJECT_NAME}_node project11_msgs_gencpp)

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


install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
