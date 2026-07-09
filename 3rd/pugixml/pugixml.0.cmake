
# this file is included automatically by the 'import_library' macro
# in the main project's scope, before using 'add_subdirectory' to
# include the sub-project.

# standard way is to ..
#    #include <pugixml.hpp>
#

include_directories ("${LIBRARIES_DIR}/${IMPORTING_LIBRARY_NAME}")
