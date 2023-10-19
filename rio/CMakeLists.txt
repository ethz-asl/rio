cmake_minimum_required(VERSION 3.0.2)
project(rio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
                message_generation
                roscpp 
                lpp 
                mav_sensors_drivers
                sensor_msgs
                std_msgs
                roslib
                tf2_eigen
                tf2_ros
                nodelet
                gtsam_catkin
                std_srvs
            )
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)

add_message_files()
generate_messages(DEPENDENCIES std_msgs)

catkin_package( INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
                LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
                CATKIN_DEPENDS message_runtime roscpp lpp mav_sensors_drivers sensor_msgs std_msgs roslib tf2_eigen tf2_ros nodelet std_srvs
              )
include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

add_compile_definitions(MODE_ROSLOG)
add_library(${PROJECT_NAME}
        src/common.cpp
        src/least_squares.cpp
        src/gtsam/landmark_tracker.cpp
        src/gtsam/optimization.cpp
        src/gtsam/propagation.cpp
        src/gtsam/state.cpp
        src/ros/base_sensor.cpp
        src/ros/baro.cpp
        src/ros/common.cpp
        src/ros/imu.cpp
        src/ros/radar.cpp
        src/ros/rio.cpp
        )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable Eigen3::Eigen)
target_compile_definitions(${PROJECT_NAME} PRIVATE GTSAM_ENABLE_DEBUG) 

add_library(${PROJECT_NAME}_nodelet
        src/ros/nodelets/baro_nodelet.cpp
        src/ros/nodelets/imu_nodelet.cpp
        src/ros/nodelets/radar_nodelet.cpp
        src/ros/nodelets/rio_nodelet.cpp
        )
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME})

add_executable(rio_node
        src/ros/nodes/rio_node.cpp
        )
target_link_libraries(rio_node ${PROJECT_NAME})

#add_executable(rio_calibration_node
#        src/ros/nodes/rio_calibration_node.cpp
#)

#########
# TESTS #
#########
if (CATKIN_ENABLE_TESTING)
        find_package(rostest REQUIRED)
        find_package(GTest REQUIRED)
        include_directories(${GTEST_INCLUDE_DIRS})

        catkin_add_gtest(test_adjoint
        test/adjoint-test.cpp
        )
        target_link_libraries(test_adjoint ${PROJECT_NAME})
endif()

###########
# INSTALL #
###########

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})