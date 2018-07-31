cmake_minimum_required(VERSION 2.8.3)
project(survey_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kongsberg_em_control
  std_msgs
  project11
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES range_and_bearing
#  CATKIN_DEPENDS std_msgs sensor_msgs project11 roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/survey_manager_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

