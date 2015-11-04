CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT ( flexrayusbinterface )

# Necessary for Qt signal/event system
set(CMAKE_AUTOMOC ON)

FIND_PACKAGE ( Qt4 COMPONENTS QtCore QtGui REQUIRED  )

INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}/include
    ${QT_INCLUDE_DIR}
    ${QT_QTCORE_INCLUDE_DIR}
    ${QT_QTGUI_INCLUDE_DIR}
)

SET( LIBS
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)

FILE(GLOB incs "include/*.h")
FILE(GLOB srcs "src/*.cpp")

ADD_EXECUTABLE( flexrayusbinterface ${incs} ${srcs})

TARGET_LINK_LIBRARIES(flexrayusbinterface ${LIBS} -lftd2xx -lpthread)
