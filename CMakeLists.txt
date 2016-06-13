CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT ( flexrayusbinterface )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

SET(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "--std=gnu++11")

# Boost
FIND_PACKAGE ( Boost COMPONENTS system thread REQUIRED  )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
SET( BOOST_LIBS
  ${Boost_LIBRARIES}
)

INCLUDE_DIRECTORIES( "/opt/ros/indigo/include")
FIND_LIBRARY(ROSCONSOLE_LIBRARY rosconsole
    /opt/ros/jade/lib
    NO_DEFAULT_PATH
)

INCLUDE_DIRECTORIES( "include/" "../common_utilities/include")

#FILE(GLOB incs "include/*.h")
FILE(GLOB incs "include/*.hpp")
FILE(GLOB srcs "src/*.cpp")

MESSAGE(STATUS ${CMAKE_SOURCE_DIR})

ADD_EXECUTABLE( flexrayusbinterface ${incs} src/FlexRayHardwareInterface.cpp src/main.cpp)

TARGET_LINK_LIBRARIES(flexrayusbinterface ${BOOST_LIBS} -lftd2xx -lpthread ${ROSCONSOLE_LIBRARY} -lncurses)
