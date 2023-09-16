cmake_minimum_required(VERSION 3.16.3)
project(mmwave_sdk_catkin)

find_package(catkin REQUIRED)
catkin_package(CFG_EXTRAS mmwave_sdk-extras.cmake)

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

set(MMWAVE_SDK_MAJOR 03)
set(MMWAVE_SDK_MINOR 06)
set(MMWAVE_SDK_PATCH 00)
set(MMWAVE_SDK_PRE_RELEASE 00)
ExternalProject_Add(mmwave_sdk
  URL https://dr-download.ti.com/software-development/software-development-kit-sdk/MD-PIrUeCYr3X/${MMWAVE_SDK_MAJOR}.${MMWAVE_SDK_MINOR}.${MMWAVE_SDK_PATCH}.${MMWAVE_SDK_PRE_RELEASE}-LTS/mmwave_sdk_${MMWAVE_SDK_MAJOR}_${MMWAVE_SDK_MINOR}_${MMWAVE_SDK_PATCH}_${MMWAVE_SDK_PRE_RELEASE}-LTS-Linux-x86-Install.bin
  URL_MD5 047f66d97fe89678553b1fef758ae09c
  DOWNLOAD_NO_EXTRACT TRUE
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND chmod +x <DOWNLOADED_FILE>
  BUILD_COMMAND 
    COMMAND <DOWNLOADED_FILE> --mode unattended --prefix <BINARY_DIR>
    COMMAND patch mmwave_sdk_${MMWAVE_SDK_MAJOR}_${MMWAVE_SDK_MINOR}_${MMWAVE_SDK_PATCH}_${MMWAVE_SDK_PRE_RELEASE}-LTS/packages/scripts/unix/setenv.sh ${CMAKE_CURRENT_SOURCE_DIR}/patches/select_awr1843.patch
  INSTALL_COMMAND ""
)

add_library(${PROJECT_NAME} src/dependency_tracker.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} mmwave_sdk)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)