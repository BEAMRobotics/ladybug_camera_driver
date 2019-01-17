function(download_ladybug FLIR_LIB_VAR FLIR_INCLUDE_DIR_VAR)
    if (NOT UNIX)
        message(FATAL_ERROR "Downloading libLadybug for non-linux systems not supported")
    endif ()

    include(cmake/TargetArch.cmake)
    target_architecture(FLIR_ARCH)
    message(STATUS "Running download_ladybug script with arguments: ${FLIR_ARCH} ${CATKIN_DEVEL_PREFIX}/lib/ladybug_camera_driver/ WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}")
    set(DOWNLOAD_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/download_ladybug")
    message(STATUS "Download script location: ${DOWNLOAD_SCRIPT}")
    execute_process(
            COMMAND ${DOWNLOAD_SCRIPT} ${FLIR_ARCH} "${CATKIN_DEVEL_PREFIX}/lib/ladybug_camera_driver/"
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    set(${FLIR_LIB_VAR} "${CMAKE_CURRENT_BINARY_DIR}/usr/lib/ladybug/libladybug.so.1" PARENT_SCOPE)
    set(${FLIR_INCLUDE_DIR_VAR} "${CMAKE_CURRENT_BINARY_DIR}/usr/include" PARENT_SCOPE)
endfunction()