cmake_minimum_required(VERSION 2.8.3)
project(ecn_bluerov)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  std_msgs
  compressed_image_transport
)

message("CMP: ${CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake/")
find_package(raspicam)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)


if(raspicam_FOUND)    # compile only on RPi
add_executable(picam ../boot/picam.cpp)
target_link_libraries(picam ${catkin_LIBRARIES} ${raspicam_CV_LIBS} ${raspicam_LIBS})

endif()
