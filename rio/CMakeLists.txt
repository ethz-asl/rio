cmake_minimum_required(VERSION 3.0.2)
project(rio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
                roscpp 
                lpp 
                mav_sensors_drivers
                sensor_msgs
                roslib
            )

catkin_package()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_compile_definitions(MODE_ROSLOG)

add_executable(radar_demo src/nodes/radar_demo.cpp)
target_link_libraries(radar_demo ${catkin_LIBRARIES})
