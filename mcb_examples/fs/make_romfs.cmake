# need this because we need to capture it for the macro, which will be
# invoked in another scope, and CMAKE_CURRENT_LIST_DIR will not point
# to this directory anymore at the time when the macro is used.
set (ADD_ROMFS_THIS_CMAKE_FILE_DIR ${CMAKE_CURRENT_LIST_DIR})

macro (add_romfs _target endianess)

# argn is everything that follows the last named argument
# for each file, 3 arguments are used:
#   file path, file name in the romfs, mime type
#
# the input files are used as dependencies on the resulting romfs image,
# thus we need to create a list containting the names only.
#
# other than that, the arguments are just passed to the make_romfs tool.
set (args ${ARGN})

list (LENGTH args len)
math (EXPR len "${len}/3 - 1")

#message ("${args}")
#message ("YYYYYY: ${len}")

set (src_files)

foreach (i RANGE ${len})

  math (EXPR ii "${i} * 3")
  list (GET args ${ii} n)
  list (APPEND src_files ${n})

endforeach (i)

# unfortunately passing '${args}' to make_romfs will always expand the semicolon
# separated list into a space separated list.  if one of the files or paths
# contains a space this will break.  thus replace the ';' in the list with '*'
string (REPLACE ";" "*" args "${args}")

#message ("src_files: ${src_files}")
#message ("args: ${args}")

#message ("current list dir: ${CMAKE_CURRENT_LIST_DIR}")
#message ("current bin dir: ${CMAKE_CURRENT_BINARY_DIR}")
#message ("this dir: ${ADD_ROMFS_THIS_CMAKE_FILE_DIR}")
#message ("source dir: ${CMAKE_SOURCE_DIR}")

add_host_tool_executable (make_romfs "" "${ADD_ROMFS_THIS_CMAKE_FILE_DIR}/make_romfs.cpp")

add_custom_command (
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${_target}.romfs_image.cpp"
  COMMAND "${CMAKE_CURRENT_BINARY_DIR}/host_tool/make_romfs" "${CMAKE_CURRENT_BINARY_DIR}/${_target}.romfs_image.cpp" ${endianess} "${CMAKE_CURRENT_LIST_DIR}" "${args}"
  DEPENDS make_romfs
  DEPENDS ${src_files}
)

add_library (${_target}.romfs_image STATIC
  ${CMAKE_CURRENT_BINARY_DIR}/${_target}.romfs_image.cpp
)

target_link_libraries (${_target}
 -Wl,-whole-archive ${_target}.romfs_image -Wl,-no-whole-archive
)

endmacro ()
