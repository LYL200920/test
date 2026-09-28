
# FIXME -- this supports only statically linked zlib for now
#          we don't have a way yet to specify whether to use shared or static
#          local libraries. assume all static for now.

if (TARGET zlibstatic)
set(ZLIB_FOUND TRUE)

# FIXME -- hardcoded path of this module
set (ZLIB_INCLUDE_DIRS ${LIBRARIES_DIR}/zlib)

# FIXME -- somehow this is not picked up properly
set (ZLIB_VERSION "1.2.13")
set (ZLIB_VERSION_STRING ${ZLIB_VERSION})

# on unix it will set the output filename to "z".
# but that will link against "-lz" which will use the system installed library.
# that's not what we want.  we want it to link against the 'zlibstatic' target.
get_property(ZLIB_LIBRARY TARGET zlibstatic PROPERTY OUTPUT_NAME)
#set (ZLIB_LIBRARY zlibstatic)

set (ZLIB_LIBRARIES ${ZLIB_LIBRARY})

set (${${ZLIB_LIBRARY}_SOURCE_DIR} ${LIBRARIES_DIR}/zlib)

endif()
