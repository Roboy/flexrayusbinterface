###############################################################################
# Find libftd2xx
#
# This sets the following variables:
# libftd2xx_FOUND - True if libftd2xx was found.
# libftd2xx_INCLUDE_DIRS - headers
# libftd2xx_LIBRARY - library

FIND_LIBRARY(libftd2xx_LIBRARY NAMES ftd2xx
	PATHS ${PROJECT_SOURCE_DIR}/third_party/ftd2xx/lib
)

FIND_PATH(libftd2xx_INCLUDE_DIRS NAMES ftd2xx.h 
	PATHS ${PROJECT_SOURCE_DIR}/third_party/ftd2xx/include
)

MARK_AS_ADVANCED(
  libftd2xx_INCLUDE_DIRS
  libftd2xx_LIBRARY)

SET( libftd2xx_FOUND "NO" )
IF(libftd2xx_INCLUDE_DIRS)
  MESSAGE(STATUS "libftd2xx include dir: ${libftd2xx_INCLUDE_DIRS}" )
  INCLUDE_DIRECTORIES( ${libftd2xx_INCLUDE_DIRS} )
  IF(libftd2xx_LIBRARY)
    MESSAGE(STATUS "libftd2xx library: ${libftd2xx_LIBRARY}" )
    SET( libftd2xx_FOUND "YES" )
  ENDIF(libftd2xx_LIBRARY)
ENDIF(libftd2xx_INCLUDE_DIRS)

IF(libftd2xx_FOUND)
    MESSAGE(STATUS "Found libftd2xx library")
ELSE(libftd2xx_FOUND)
    MESSAGE(FATAL_ERROR "Could not find libftd2xx")
ENDIF(libftd2xx_FOUND)
