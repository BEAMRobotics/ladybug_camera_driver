cmake_minimum_required(VERSION 3.0)
project(ladybug_camera_driver)

add_compile_options(-std=c++14)# -Wall -Werror)

#set(BOOST_ROOT /usr/local/include/boost)
#set(BOOST_LIBRARYDIR /usr/local/lib)
find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  diagnostic_updater
  dynamic_reconfigure
  image_exposure_msgs
  image_transport
  nodelet
  roscpp
  sensor_msgs
  ladybug_msgs
  roslaunch
  )

find_package(OpenCV 3 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
find_package(DC1394 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Ladybug.cfg
)

# If Ladybug is already present, use the found version. If not, download it.
# We can't resolve this dependency using the usual rosdep means because
# the Point Grey EULA prohibits redistributing the headers or the packages which
# contains them. We work around this by downloading the archive directly from
# their website during this step in the build process.

find_package(Ladybug QUIET)
if (NOT Ladybug_FOUND)
  message(STATUS "libLadybug not found in system library path")
  include(cmake/DownloadLadybugSDK.cmake)
  message(STATUS "debug_10---------------------")
  download_ladybug(Ladybug_LIBRARIES Ladybug_INCLUDE_DIRS)
endif ()

message(STATUS "Ladybug found ${ladybug_FOUND}")
message(STATUS "libLadybug library: ${Ladybug_LIBRARIES}")
message(STATUS "libLadybug include: ${Ladybug_INCLUDE_DIRS}")

catkin_package(
  INCLUDE_DIRS
  include
  ${Ladybug_INCLUDE_DIRS}
  LIBRARIES
  ${Ladybug_LIBRARIES}
  CATKIN_DEPENDS
  image_exposure_msgs
  nodelet
  roscpp
  sensor_msgs
  ladybug_msgs
  DEPENDS
  ${Ladybug_LIBRARIES}
)

#link_directories(/home/steve/dummy_ws/build/ladybug/ladybug_camera_driver/usr/lib/ladybug)

#Include directories
include_directories(include
  ${Ladybug_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${DC1394_INCLUDE_DIRS})

# Build LadybugCamera wrapper library
add_library(LadybugCamera
  src/LadybugRectified.cpp
  src/LadybugInterface.cpp
  src/LadybugRaw.cpp
)


message(STATUS "ladybug library directory: ${Ladybug_LIBRARIES}")


target_link_libraries(LadybugCamera
  ${Ladybug_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${DC1394_LIBRARIES})

add_dependencies(LadybugCamera ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_library(Diagnostics src/diagnostics.cpp)
target_link_libraries(Diagnostics LadybugCamera ${catkin_LIBRARIES})
add_dependencies(Diagnostics ${PROJECT_NAME}_gencfg)

add_library(LadybugCameraNodelet src/LadybugNodelet.cpp)
target_link_libraries(LadybugCameraNodelet
  Diagnostics
  LadybugCamera
  ${catkin_LIBRARIES})
add_dependencies(LadybugCameraNodelet ${catkin_EXPORTED_TARGETS})

add_executable(ladybug_camera_node src/ladybug_camera_node.cpp)
target_link_libraries(ladybug_camera_node
  LadybugCamera
  ${catkin_LIBRARIES})
set_target_properties(ladybug_camera_node
  PROPERTIES OUTPUT_NAME camera_node PREFIX "")
add_dependencies(ladybug_camera_node ${catkin_EXPORTED_TARGETS})


install(TARGETS
  LadybugCamera
  LadybugCameraNodelet
  Diagnostics
  ladybug_camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Redistributing the ladybug .so file is permitted by the SDK EULA:
# http://www.ptgrey.com/support/kb/data/PGR-FlyCap-SDK-LA.pdf --------------_CHECK
#install(FILES ${Ladybug_LIBRARIES} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Redistributing the flycapture .so file is permitted by the SDK EULA:
# http://www.ptgrey.com/support/kb/data/PGR-FlyCap-SDK-LA.pdf
#install(FILES ${POINTGREY_LIB} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#set(CATKIN_ENABLE_TESTING True)

find_package(roslaunch REQUIRED)
roslaunch_add_file_check(launch/camera.launch)

#if (CATKIN_ENABLE_TESTING)
#    find_package(roslint REQUIRED)
#    roslint_cpp()
#endif ()
