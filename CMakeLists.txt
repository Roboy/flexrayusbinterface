CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT ( flexrayusbinterface )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

SET(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "--std=gnu++11")

FIND_PACKAGE(catkin REQUIRED COMPONENTS 
    common_utilities
    rosconsole
    roscpp)

#FIND_PACKAGE (ncurses REQUIRED)

# Boost
FIND_PACKAGE ( Boost COMPONENTS system thread REQUIRED  )
SET( BOOST_LIBS
  ${Boost_LIBRARIES}
)



INCLUDE_DIRECTORIES(    include 
                        ${Boost_INCLUDE_DIR}
                        ${catkin_INCLUDE_DIRS})


MESSAGE(STATUS ${CMAKE_SOURCE_DIR})


catkin_package(DEPENDS Boost
               INCLUDE_DIRS include
               CATKIN_DEPENDS common_utilities roscpp rosconsole
               LIBRARIES libflexrayusbinterface)
               
ADD_EXECUTABLE( flexrayusbinterface  
                        src/FlexRayHardwareInterface.cpp src/main.cpp)

TARGET_LINK_LIBRARIES( flexrayusbinterface 
                        ${BOOST_LIBS} 
                        -lftd2xx 
                        -lpthread 
                        -lncurses 
                        ${catkin_LIBRARIES}
)

ADD_LIBRARY ( libflexrayusbinterface ${incs}  
                        src/FlexRayHardwareInterface.cpp)

TARGET_LINK_LIBRARIES( libflexrayusbinterface 
                        ${BOOST_LIBS}
                        -lftd2xx
                        -lpthread
                        ${ROSCONSOLE_LIBRARY}
                        ${catkin_LIBRARIES}
)