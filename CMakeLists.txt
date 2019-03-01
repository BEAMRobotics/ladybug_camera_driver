cmake_minimum_required(VERSION 3.0)
project(ladybug_camera_driver)

set(build_depends
  camera_info_manager
  diagnostic_updater
  dynamic_reconfigure
  image_exposure_msgs
  image_transport
  ladybug_msgs
  nodelet
  roscpp
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
)

find_package(OpenCV 3 REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
find_package(Ladybug QUIET)
if (NOT Ladybug_FOUND)
  include(cmake/DownloadLadybugSDK.cmake)
  download_ladybug(Ladybug_LIBRARIES Ladybug_INCLUDE_DIRS)
endif ()

## Generate dynamic reconfig messages
generate_dynamic_reconfigure_options(
  cfg/Ladybug.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
    ${Ladybug_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    ${Ladybug_LIBRARIES}
  CATKIN_DEPENDS
    ${build_depends}
  DEPENDS
    ${Ladybug_LIBRARIES}
)

###########
## Build ##
###########
add_compile_options(-std=c++17)

# ladybug_camera_driver library
add_library(${PROJECT_NAME}
  src/diagnostics.cpp
  src/LadybugInterface.cpp
  src/LadybugRaw.cpp
  src/LadybugRectified.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${Ladybug_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Ladybug_LIBRARIES}
  ${OpenCV_LIBS}
)

# Build camera nodelet
add_library(LadybugCameraNodelet
  src/LadybugNodelet.cpp
)
add_dependencies(LadybugCameraNodelet
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(LadybugCameraNodelet
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(LadybugCameraNodelet
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# ladybug_camera_node
add_executable(ladybug_camera_node
  src/ladybug_camera_node.cpp
)
add_dependencies(ladybug_camera_node
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(ladybug_camera_node
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(ladybug_camera_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## INSTALL ##
#############
install(TARGETS ${PROJECT_NAME} LadybugCameraNodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ladybug_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)