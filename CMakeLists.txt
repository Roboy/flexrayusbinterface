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

FIND_PACKAGE (PkgConfig REQUIRED)
pkg_check_modules(ncurses REQUIRED ncurses)

# Boost
FIND_PACKAGE ( Boost COMPONENTS system thread REQUIRED  )

find_package(libFTD2XX)



INCLUDE_DIRECTORIES(    include
                        ${Boost_INCLUDE_DIR}
                        ${catkin_INCLUDE_DIRS}
                        ${ncurses_INCLUDE_DIRS}
                        )


MESSAGE(STATUS ${CMAKE_SOURCE_DIR})


catkin_package(DEPENDS Boost ncurses
               INCLUDE_DIRS include
               CATKIN_DEPENDS common_utilities roscpp rosconsole
               LIBRARIES libflexrayusbinterface
             DEPENDS libftd2xx)

ADD_EXECUTABLE( flexrayusbinterface
                        src/FlexRayHardwareInterface.cpp src/main.cpp)

TARGET_LINK_LIBRARIES( flexrayusbinterface
                        ${Boost_LIBRARIES}
                        -lftd2xx
                        -lpthread
                        ${catkin_LIBRARIES}
                        ${ncurses_LIBRARIES}
                        ${libftd2xx_LIBRARY}

)
add_dependencies(flexrayusbinterface std_msgs_generate_messages_cpp common_utilities_generate_messages_cpp)

ADD_LIBRARY ( libflexrayusbinterface ${incs}
                        src/FlexRayHardwareInterface.cpp)

TARGET_LINK_LIBRARIES( libflexrayusbinterface
                        ${Boost_LIBRARIES}
                        -lftd2xx
                        -lpthread
                        ${ROSCONSOLE_LIBRARIES}
                        ${catkin_LIBRARIES}
                        ${ncurses_LIBRARIES}
                          ${libftd2xx_LIBRARY}
)
add_dependencies(libflexrayusbinterface std_msgs_generate_messages_cpp common_utilities_generate_messages_cpp)
