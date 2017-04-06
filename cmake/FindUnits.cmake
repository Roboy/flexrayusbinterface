###############################################################################
# Find Units
#
# This sets the following variables:
# Units_FOUND - True if Units was found.
# Units_INCLUDE_DIRS - headers


find_path(Units_INCLUDE_DIR NAMES units.h
	PATHS ${PROJECT_SOURCE_DIR}/../flexrayusbinterface/third_party/include/ 
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Units  DEFAULT_MSG Units_INCLUDE_DIR)

MARK_AS_ADVANCED(Units_INCLUDE_DIR)

SET(Units_INCLUDE_DIRS "${Units_INCLUDE_DIR}")
