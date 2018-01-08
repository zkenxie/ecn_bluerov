cmake_minimum_required(VERSION 2.8.3)
project(ecn_bluerov)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
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
  Navio2/C++/Navio
  ${catkin_INCLUDE_DIRS}
)

file(GLOB navio2_SRC
    "Navio2/C++/Navio/Navio2/*.h"
    "Navio2/C++/Navio/Common/*.h"
    "Navio2/C++/Navio/Navio2/*.cpp"
    "Navio2/C++/Navio/Common/*.cpp"
)
message("Navio2: " ${navio2_SRC})
add_library(navio2 ${navio2_SRC})

add_executable(pwm boot/pwm.cpp)
target_link_libraries(pwm navio2 ${catkin_LIBRARIES})
