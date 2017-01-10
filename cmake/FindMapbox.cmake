###############################################################################
# Find Mapbox
#
# This sets the following variables:
# Mapbox_FOUND - True if Mapbox was found.
# Mapbox_INCLUDE_DIRS - headers


find_path(Mapbox_INCLUDE_DIR NAMES mapbox/variant.hpp
	PATHS ${PROJECT_SOURCE_DIR}/../flexrayusbinterface/lib/Mapbox/include/ 
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Mapbox  DEFAULT_MSG Mapbox_INCLUDE_DIR)

MARK_AS_ADVANCED(Mapbox_INCLUDE_DIR)

SET(Mapbox_INCLUDE_DIRS "${Mapbox_INCLUDE_DIR}")

