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
find_package(Eigen3 REQUIRED)

catkin_package( INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
                LIBRARIES ${PROJECT_NAME}
              )
include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

add_compile_definitions(MODE_ROSLOG)
add_library(${PROJECT_NAME}
        src/least_squares.cpp
        )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(baro_demo src/nodes/baro_demo.cpp)
target_link_libraries(baro_demo ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(radar_demo src/nodes/radar_demo.cpp)
target_link_libraries(radar_demo ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})