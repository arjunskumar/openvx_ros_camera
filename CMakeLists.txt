cmake_minimum_required(VERSION 3.0.2)
project(openvx_ros_camera)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 3.3.1 REQUIRED)  

#include_directories("/home/arjun/OpenVX-sample-impl/install/Linux/x64/Debug/include")

set(OPENVX_INCLUDES "/home/arjun/OpenVX-sample-impl/install/Linux/x64/Debug/include" ${OPENVX_INCLUDES})  
set(OPENVX_LIBRARIES "/home/arjun/OpenVX-sample-impl/install/Linux/x64/Debug/bin/libopenvx.so" ${OPENVX_LIBRARIES})  


if(DEFINED OPENVX_INCLUDES)
    message(STATUS "OPENVX_INCLUDES: \t${OPENVX_INCLUDES}")
    include_directories(${OPENVX_INCLUDES})
else()
    message(STATUS "OPENVX_INCLUDES: \t<empty>")
endif()

if(DEFINED OPENVX_LIBRARIES)
    message(STATUS "OPENVX_LIBRARIES: \t${OPENVX_LIBRARIES}")
else()
    message(STATUS "OPENVX_LIBRARIES: \t<empty>")
endif()

catkin_package()


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cameraproc src/camera_test.cpp) 

target_link_libraries(cameraproc
  ${catkin_LIBRARIES}
  ${OPENVX_LIBRARIES}
  ${OpenCV_LIBRARIES}
)