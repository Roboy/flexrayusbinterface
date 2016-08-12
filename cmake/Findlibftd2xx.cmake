###############################################################################
# Find libftd2xx
#
# This sets the following variables:
# libftd2xx_FOUND - True if libftd2xx was found.
# libftd2xx_INCLUDE_DIRS - headers
# libftd2xx_LIBRARY - library

FIND_LIBRARY(libftd2xx_LIBRARY ftd2xx
	${PROJECT_SOURCE_DIR}/../flexrayusbinterface/lib/libftd2xx-x86_64-1.3.6/build/
	NO_DEFAULT_PATH
)

SET(libftd2xx_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../flexrayusbinterface/lib/libftd2xx-x86_64-1.3.6/)
INCLUDE_DIRECTORIES( ${libftd2xx_INCLUDE_DIRS} )

MARK_AS_ADVANCED(
  libftd2xx_INCLUDE_DIRS
  libftd2xx_LIBRARY)

SET( libftd2xx_FOUND "NO" )
IF(libftd2xx_INCLUDE_DIRS)
MESSAGE(STATUS "libftd2xx include dir: ${libftd2xx_INCLUDE_DIRS}" )
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
