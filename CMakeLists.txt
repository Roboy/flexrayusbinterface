CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

project ( flexrayusbinterface )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "--std=gnu++11 -g")
#==============================================================================
# Dependencies
#==============================================================================
#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp )
catkin_package(CATKIN_DEPENDS message_runtime)

# Boost
find_package ( Boost COMPONENTS system thread REQUIRED  )
include_directories( ${Boost_INCLUDE_DIR} )

# pthread
find_package (Threads)

# libftd2xx
find_package(libftd2xx)

# ncurses
find_package(PkgConfig)
pkg_check_modules(ncurses REQUIRED ncurses)

include_directories( ${catkin_INCLUDE_DIRS}
        include/
        ../common_utilities/include
        )
#==============================================================================
# Application
#==============================================================================
add_executable( flexrayusbinterface src/FlexRayHardwareInterface.cpp src/main.cpp)
target_link_libraries(flexrayusbinterface ${Boost_LIBRARIES} ${libftd2xx_LIBRARY} ${catkin_LIBRARIES}
        ${ncurses_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} -ldl )
#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        flexrayusbinterface
	    RUNTIME DESTINATION bin
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

