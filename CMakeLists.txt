cmake_minimum_required(VERSION 3.0.2)
project(controller_tutorial)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  potbot_lib
  potbot_msgs
  interactive_markers
)

generate_dynamic_reconfigure_options(
  cfg/controller_tutorial.cfg
  cfg/controller_test.cfg
)

catkin_package(
  LIBRARIES potbot_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/controller_tutorial.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_test src/controller_test.cpp)
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_joy_to_cmd src/joy_to_cmd.cpp)
target_link_libraries(${PROJECT_NAME}_joy_to_cmd ${catkin_LIBRARIES})

add_executable(follow_marker src/follow_marker.cpp)
target_link_libraries(follow_marker ${catkin_LIBRARIES})

add_definitions(-std=c++14)