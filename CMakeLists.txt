cmake_minimum_required(VERSION 3.0.2)
project(ias_unitree_camera)

find_package(catkin REQUIRED COMPONENTS
  #  image_transport
  #  cv_bridge
  roscpp
  rospy
  std_msgs
  pcl_ros
  message_generation
  #  camera_info_manager
)

catkin_package(
    #CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs
    CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

 
find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIR})
    message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")
    message(STATUS ${OpenCV_LIBS})
    if(${OpenCV_VERSION_MAJOR} EQUAL 4)
        add_definitions(-DOPENCV_VERSION_4)
    elseif(${OpenCV_VERSION_MAJOR} LESS 4)
        add_definitions(-DOPENCV_VERSION_3)
    endif()
else()
    message(FATAL_ERROR "OpenCV Library Not Found")
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/amd64/)
link_directories(${PROJECT_SOURCE_DIR}/lib/amd64/)
else()
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/arm64/)
link_directories(${PROJECT_SOURCE_DIR}/lib/arm64/)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

