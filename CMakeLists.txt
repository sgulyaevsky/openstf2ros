cmake_minimum_required(VERSION 2.8.3)
project(openstf2ros)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  roslaunch
)

find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/tiny-process-library)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tiny-process-library/
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        src/openstf2ros_node.cpp
        src/openstf_init.cpp
        src/minicap.cpp
        src/minitouch.cpp
        )

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        tiny-process-library
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        )