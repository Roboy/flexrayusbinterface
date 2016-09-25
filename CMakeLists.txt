CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT ( flexrayusbinterface )

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS "--std=gnu++11 -g")
#==============================================================================
# Dependencies
#==============================================================================
# Boost
FIND_PACKAGE ( Boost COMPONENTS system thread REQUIRED  )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
SET( BOOST_LIBS
  ${Boost_LIBRARIES}
)

INCLUDE_DIRECTORIES( "/opt/ros/jade/include")
FIND_LIBRARY(ROSCONSOLE_LIBRARY rosconsole
    /opt/ros/jade/lib
    NO_DEFAULT_PATH
)

# libftd2xx
FIND_PACKAGE(libftd2xx)

INCLUDE_DIRECTORIES( "include/" "../common_utilities/include")
#==============================================================================
# Application
#==============================================================================
ADD_EXECUTABLE( flexrayusbinterface src/FlexRayHardwareInterface.cpp src/main.cpp)
TARGET_LINK_LIBRARIES(flexrayusbinterface ${BOOST_LIBS} ${libftd2xx_LIBRARY} -ldl -lpthread ${ROSCONSOLE_LIBRARY} -lncurses)
#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        flexrayusbinterface
	RUNTIME DESTINATION bin
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

