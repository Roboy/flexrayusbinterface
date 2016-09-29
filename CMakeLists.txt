CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

project ( flexrayusbinterface )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "--std=gnu++11 -g")
#==============================================================================
# Dependencies
#==============================================================================
# Boost
find_package ( Boost COMPONENTS system thread REQUIRED  )
include_directories( ${Boost_INCLUDE_DIR} )

# ros console
include_directories( "/opt/ros/jade/include")
find_library(ROSCONSOLE_LIBRARY rosconsole
    /opt/ros/jade/lib
    NO_DEFAULT_PATH
)

# pthread
find_package (Threads)

# libftd2xx
find_package(libftd2xx)

# ncurses
find_package(PkgConfig)
pkg_check_modules(ncurses REQUIRED ncurses ncursesw)

include_directories( "include/" "../common_utilities/include")
#==============================================================================
# Application
#==============================================================================
add_executable( flexrayusbinterface src/FlexRayHardwareInterface.cpp src/main.cpp)
target_link_libraries(flexrayusbinterface ${Boost_LIBRARIES} ${libftd2xx_LIBRARY} ${ROSCONSOLE_LIBRARY}
        ${ncurses_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        flexrayusbinterface
	    RUNTIME DESTINATION bin
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

