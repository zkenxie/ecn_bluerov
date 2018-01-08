cmake_minimum_required(VERSION 2.8.3)
project(ecn_bluerov)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ecn_bluerov
#  CATKIN_DEPENDS cv_bridge sensor_msgs std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  Navio2/C++/Navio/Navio2
  Navio2/C++/Navio/Common
  ${catkin_INCLUDE_DIRS}
)

file(GLOB navio2_SRC
    "Navio2/C++/Navio/Navio2/*.h"
    "Navio2/C++/Navio/Common/*.cpp"
)
add_library(navio2 ${navio2_SRC})
