CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT ( flexrayusbinterface )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

SET(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "--std=gnu++11")
#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS
    common_utilities
    rosconsole
    roscpp)

find_package (PkgConfig REQUIRED)
pkg_check_modules(ncurses REQUIRED ncurses)

# Boost
find_package ( Boost COMPONENTS system thread REQUIRED  )

# libftd2xx driver lib
find_package(libftd2xx REQUIRED)

# pthread
find_package (Threads)

catkin_package(DEPENDS Boost ncurses
        INCLUDE_DIRS include
        CATKIN_DEPENDS common_utilities roscpp rosconsole
        LIBRARIES libflexrayusbinterface
        DEPENDS libftd2xx)


INCLUDE_DIRECTORIES(    include
                        ../common_utilities/include
                        ${libftd2xx_INCLUDE_DIRS}
                        ${Boost_INCLUDE_DIR}
                        ${catkin_INCLUDE_DIRS}
                        ${ncurses_INCLUDE_DIRS}
                        )

#==============================================================================
# Application
#==============================================================================
add_library ( libflexrayusbinterface src/FlexRayHardwareInterface.cpp src/Spi.cpp)
add_dependencies(libflexrayusbinterface std_msgs_generate_messages_cpp common_utilities_generate_messages_cpp)
target_link_libraries( libflexrayusbinterface
                        ${Boost_LIBRARIES}
                        ${libftd2xx_LIBRARY}
                        ${CMAKE_THREAD_LIBS_INIT}
                        ${ROSCONSOLE_LIBRARIES}
                        ${catkin_LIBRARIES}
                        ${ncurses_LIBRARIES}
                        -ldl
)
#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        libflexrayusbinterface
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
