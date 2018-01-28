cmake_minimum_required(VERSION 3.0.2)
project(ias_unitree_camera)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs cv_bridge 
)

# -------OPENCV---------
find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")
  message(STATUS ${OpenCV_INCLUDE_DIRS})
  message(STATUS ${OpenCV_LIBS})
  if(${OpenCV_VERSION_MAJOR} EQUAL 4)
    add_definitions(-DOPENCV_VERSION_4)
  elseif(${OpenCV_VERSION_MAJOR} LESS 4)
    add_definitions(-DOPENCV_VERSION_3)
  endif()
else()
  message(FATAL_ERROR "OpenCV Library Not Found")
endif()

# -------PCL---------
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# -------UNITREE LIBRARIES---------
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  # set(LIBRARY_OUTPUT_PATH lib/amd64/)
  link_directories(lib/amd64/)
else()
  # set(LIBRARY_OUTPUT_PATH lib/arm64/)
  link_directories(lib/arm64/)
endif()

include_directories(
    include/${PROJECT_NAME}/
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

set(SDK_LIBRARIES unitree_camera tstc_V4L2_xu_camera udev systemlog ${OpenCV_LIBS})

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES} ${SDK_LIBRARIES} ${PCL_LIBRARIES})

install(
  TARGETS camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)