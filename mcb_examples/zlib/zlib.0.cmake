
# this file is included automatically by the 'import_library' macro
# in the main project's scope, before using 'add_subdirectory' to
# include the sub-project.

# set some sensible defaults for inclusion of zlib as a sub-project.

# standard way is to ..
#    include <zlib.h>
#
# .. so we need to add the whole zlib directory to the include dirs

include_directories ("${LIBRARIES_DIR}/${IMPORTING_LIBRARY_NAME}")

# for generated zconf.h
include_directories ("${CMAKE_CURRENT_BINARY_DIR}/${IMPORTING_LIBRARY_NAME}")
