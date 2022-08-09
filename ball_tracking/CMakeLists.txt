cmake_minimum_required(VERSION 3.0.2)
project(ball_tracking)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  roslib
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp rospy roslib cv_bridge sensor_msgs geometry_msgs std_msgs
DEPENDS system_lib OPENCV EIGEN3
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(ball_tracking src/ball_tracking_node.cpp src/Tracking.cpp src/Kalman.cpp)
target_link_libraries(ball_tracking ${catkin_LIBRARIES} ${OpenCV_LIBS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${OPENCV_LIBRARIES})
add_dependencies(ball_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
