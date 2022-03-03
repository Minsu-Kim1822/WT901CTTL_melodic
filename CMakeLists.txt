cmake_minimum_required(VERSION 3.0.2)
project(wit_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp
  serial
  sensor_msgs
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/wit_driver_node.cpp src/JY901.cpp)

message(${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
