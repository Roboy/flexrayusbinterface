CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT ( flexrayusbinterface )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../flexrayusbinterface/cmake)

SET(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "-std=c++11")
#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS)

find_package (PkgConfig REQUIRED)

# Boost
find_package ( Boost REQUIRED COMPONENTS thread )

# Mapbox
find_package ( Mapbox REQUIRED )

# Mapbox
find_package ( Units REQUIRED )

# yaml-cpp
find_package ( yaml-cpp REQUIRED )

# libftd2xx driver lib
find_package(libftd2xx REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES libflexrayusbinterface
        DEPENDS libftd2xx Mapbox Units Boost YAML_CPP)


include_directories(
    include
    ${libftd2xx_INCLUDE_DIRS}
    ${Mapbox_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${Units_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
    )

#==============================================================================
# Library
#==============================================================================
add_library(libflexrayusbinterface
    src/FlexRayHardwareInterface.cpp
    src/Spi.cpp
    src/UsbChannel.cpp
    src/Calibration.cpp
    src/Muscle.cpp
)

target_link_libraries( libflexrayusbinterface ${libftd2xx_LIBRARY} -ldl)
target_link_libraries(libflexrayusbinterface ${YAML_CPP_LIBRARIES} -ldl)
target_link_libraries(libflexrayusbinterface ${Boost_LIBRARIES})

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        libflexrayusbinterface
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

add_executable(list_ftdi_devices src/ListDevices.cpp)
target_link_libraries(list_ftdi_devices ${libftd2xx_LIBRARY})

install(TARGETS
        list_ftdi_devices
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

set(TEST_SOURCE_FILES
    test/Main.cpp
    test/Message.cpp
    test/Protocol.cpp
    test/Slot.cpp
    test/FlexRayBus.cpp
    test/yaml/Parsers.cpp
)
catkin_add_gtest(unit_tests ${TEST_SOURCE_FILES})
target_link_libraries(unit_tests ${Boost_LIBRARIES})
target_link_libraries(unit_tests ${YAML_CPP_LIBRARIES})
configure_file(test/yaml/data/RobotDescription.yaml ${CMAKE_CURRENT_BINARY_DIR}/RobotDescription.yaml COPYONLY)
