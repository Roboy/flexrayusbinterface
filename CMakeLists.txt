CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(DCMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

PROJECT ( flexrayusbinterface )

## Necessary for Qt signal/event system
#set(CMAKE_AUTOMOC ON)
#
## QT
#FIND_PACKAGE ( Qt4 COMPONENTS QtCore QtGui REQUIRED  )
#INCLUDE_DIRECTORIES(
#    ${CMAKE_SOURCE_DIR}/include
#    ${QT_INCLUDE_DIR}
#    ${QT_QTCORE_INCLUDE_DIR}
#    ${QT_QTGUI_INCLUDE_DIR}
#)
#SET( QT_LIBS
#  ${QT_QTCORE_LIBRARY}
#  ${QT_QTGUI_LIBRARY}
#)

# Boost
FIND_PACKAGE ( Boost COMPONENTS system thread REQUIRED  )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
SET( BOOST_LIBS
  ${Boost_LIBRARIES}
)

INCLUDE_DIRECTORIES( "include/")

#FILE(GLOB incs "include/*.h")
FILE(GLOB incs "include/*.hpp")
FILE(GLOB srcs "src/*.cpp")

MESSAGE(STATUS ${CMAKE_SOURCE_DIR})

ADD_EXECUTABLE( flexrayusbinterface ${incs} src/FlexRayHardwareInterface.cpp src/main.cpp)

TARGET_LINK_LIBRARIES(flexrayusbinterface ${BOOST_LIBS} -lftd2xx -lpthread)
