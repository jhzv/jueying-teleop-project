cmake_minimum_required(VERSION 3.0.2)
project(teleop_jueying)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs diagnostic_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_orchestrator src/teleop_orchestrator.cpp)
target_link_libraries(teleop_orchestrator ${catkin_LIBRARIES})

install(TARGETS teleop_orchestrator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
