unset(Ladybug_FOUND)
unset(Ladybug_INCLUDE_DIRS)
unset(Ladybug_LIBRARIES)

message(STATUS "Inside FindLadybug.cmake")
find_path(Ladybug_INCLUDE_DIRS NAMES
        ladybug.h
        PATHS
        /usr/include/ladybug/
        /usr/local/include/ladybug/
        )

find_library(Ladybug_LIBRARIES NAMES ladybug
        PATHS
        /usr/lib/ladybug/
        /usr/local/lib/ladybug/
        )

if (Ladybug_INCLUDE_DIRS AND Ladybug_LIBRARIES)
    set(Ladybug_FOUND 1)
endif (Ladybug_INCLUDE_DIRS AND Ladybug_LIBRARIES)