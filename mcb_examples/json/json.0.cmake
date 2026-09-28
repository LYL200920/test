
# this file is included automatically by the 'import_library' macro
# in the main project's scope, before using 'add_subdirectory' to
# include the sub-project.

# set some sensible defaults for inclusion of nlohmann json as a sub-project.

if (NOT DEFINED JSON_BuildTests)
  set (JSON_BuildTests OFF CACHE BOOL "")
endif ()

# standard way is to ..
#    #include <nlohmann/json.hpp>
#

include_directories ("${LIBRARIES_DIR}/${IMPORTING_LIBRARY_NAME}/include")
