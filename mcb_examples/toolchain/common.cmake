# common functions, macros, etc shared by all toolchain descriptions.

# ----------------------------------------------------------------------------
# for LibFindMacros.cmake and project specific find-module cmake files

list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

# ----------------------------------------------------------------------------

if (NOT CMAKE_SYSTEM_PROCESSOR)
  set (CMAKE_SYSTEM_PROCESSOR ${CMAKE_HOST_SYSTEM_PROCESSOR})
endif ()

# always reset the executable path to the default.
# when cmake is ran repeatedly, some libraries might leave it in
# a modified state
set (EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# ----------------------------------------------------------------------------

# mark some of the variables introduced here as "internal".
# could also use "mark_as_advanced" ...
macro (mark_as_internal _var)
  set ( ${_var} ${${_var}} CACHE INTERNAL "" FORCE )
endmacro ()

# ----------------------------------------------------------------------------

if (MSVC)

set (BUILD_PROCESSOR_COUNT "1" CACHE STRING "Build Processor Count")

else ()

  include (ProcessorCount)

  set (BUILD_PROCESSOR_COUNT "" CACHE STRING "Build Processor Count")

  if (NOT BUILD_PROCESSOR_COUNT OR BUILD_PROCESSOR_COUNT STREQUAL "")
    ProcessorCount (BUILD_PROCESSOR_COUNT)
    set (BUILD_PROCESSOR_COUNT "${BUILD_PROCESSOR_COUNT}" CACHE STRING "Build Processor Count" FORCE)
    message ("build processor count set to ${BUILD_PROCESSOR_COUNT}")
  endif ()

endif ()

# ----------------------------------------------------------------------------
# import a local library that is in the {LIBRARIES_DIR} directory
#
# if the library project adds more include directories (e.g. such as
# some external libraries like zlib do), those will be collected and
# added to the xxx_INCLUDE_DIR variable which find_package can pick up.
# this avoids adding those additional includes to the project scope,
# although that would also work.
# this makes it easier to include 3rd party libraries which do have a cmake
# script but are not designed for inclusion into another project.

macro (import_library _name)

if (NOT TARGET ${_name})

if (CMAKE_CXX_STANDARD)
  set (PREV_CMAKE_CXX_STANDARD ${CMAKE_CXX_STANDARD})
endif ()

if (CMAKE_CXX_STANDARD_REQUIRED)
  set (PREV_CMAKE_CXX_STANDARD_REQUIRED ${CMAKE_CXX_STANDARD_REQUIRED})
endif ()

if (CMAKE_CXX_EXTENSIONS)
  set (PREV_CMAKE_CXX_EXTENSIONS ${CMAKE_CXX_EXTENSIONS})
endif ()

set (IMPORTING_LIBRARY TRUE CACHE INTERNAL "")
set (IMPORTING_LIBRARY_NAME "${_name}" CACHE INTERNAL "")
string (TOUPPER ${_name} LIB_PREFIX)

# if the library sub-project wants to create executables, then it should
# do that in its own sub-directory
set (PREV_EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH})
set (EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/${IMPORTING_LIBRARY_NAME})

if ((${_name} STREQUAL "board") AND (BOARD_DIR))

  add_subdirectory (${BOARD_DIR} "${CMAKE_CURRENT_BINARY_DIR}/board")

else ()

  if (LIBRARIES_DIR)

    # if the library contains a 'FindXXXX.cmake' allow cmake to find them
    list (APPEND CMAKE_MODULE_PATH "${LIBRARIES_DIR}/${_name}")

    include (${LIBRARIES_DIR}/${_name}/${_name}.0.cmake OPTIONAL)

    # some standalone libraries refer to their xxx_SOURCE_DIR and
    # xxx_BINARY_DIR which is not set when projects include other projects
    set (${LIB_PREFIX}_SOURCE_DIR "${LIBRARIES_DIR}/${_name}" CACHE INTERNAL "")
    set (${LIB_PREFIX}_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/${_name}" CACHE INTERNAL "")

    get_filename_component (libs_dir "${LIBRARIES_DIR}/${_name}/../" ABSOLUTE)
    include_directories (${libs_dir})

    # before we include the subdirectory its INCLUDE_DIRECTORIES property does
    # not exist yet.  so we use this scope's INCLUDE_DIRECTORIES value as a start.
    get_property (dirs_before DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

    add_subdirectory (${LIBRARIES_DIR}/${_name} "${CMAKE_CURRENT_BINARY_DIR}/${_name}")

    include (${LIBRARIES_DIR}/${_name}/${_name}.cmake OPTIONAL)

    get_property (dirs_after DIRECTORY "${LIBRARIES_DIR}/${_name}" PROPERTY INCLUDE_DIRECTORIES)

    # get rid of duplicated entries
    list (REMOVE_ITEM dirs_after ${dirs_before})
    #message ("import library ${_name} dirs_before: ${dirs_before}")
    #message ("import library ${_name} dirs_after: ${dirs_after}")

    # don't add to parent project scope, only to xxx_INCLUDE_DIR
    #include_directories (${dirs_after})

    # find_package looks for xxx_LIBRARY and xxx_INCLUDE_DIR variables
    if (NOT ${LIB_PREFIX}_LIBRARY AND NOT ${LIB_PREFIX}_LIBRARIES)
      set (${LIB_PREFIX}_LIBRARY ${_name} CACHE INTERNAL "")
      set (${LIB_PREFIX}_LIBRARIES ${${LIB_PREFIX}_LIBRARY} CACHE INTERNAL "")
    endif ()

    if (NOT ${LIB_PREFIX}_INCLUDE_DIR AND NOT ${LIB_PREFIX}_INCLUDE_DIRS)
      set (${LIB_PREFIX}_INCLUDE_DIR "${LIBRARIES_DIR}/${_name};${dirs_after}" CACHE INTERNAL "")
      set (${LIB_PREFIX}_INCLUDE_DIRS ${${LIB_PREFIX}_INCLUDE_DIR} CACHE INTERNAL "")
    endif ()

    get_property (incdirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    list (REMOVE_DUPLICATES incdirs)
    set_property (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES ${incdirs})

    rebuild_build_flags ()

  else ()

    message (FATAL_ERROR "LIBRARIES_DIR not set, can't use import_library")

  endif ()

  message (STATUS "import_library -- ${IMPORTING_LIBRARY_NAME} defined targets:")

  unset (__imported_library__target_names)
    if (EXISTS "${LIBRARIES_DIR}/${IMPORTING_LIBRARY_NAME}")
    get_directory_property (__imported_library__target_names DIRECTORY "${LIBRARIES_DIR}/${IMPORTING_LIBRARY_NAME}" BUILDSYSTEM_TARGETS)
  endif ()

  foreach (__imported_library_target_name ${__imported_library__target_names})

    get_property (__imported_target_type
                  TARGET ${__imported_library_target_name}
                  PROPERTY TYPE)

    message (STATUS "     ${__imported_library_target_name}: ${__imported_target_type}")

  endforeach ()

  # append the library source directory to the module search path so that
  # find_package will look inside the library's directory for the "Find<module>.cmake" file.
  set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${LIBRARIES_DIR}/${_name}")

endif ()


set (EXECUTABLE_OUTPUT_PATH ${PREV_EXECUTABLE_OUTPUT_PATH})

if (PREV_CMAKE_CXX_STANDARD)
  set (CMAKE_CXX_STANDARD ${PREV_CMAKE_CXX_STANDARD})
endif ()

if (PREV_CMAKE_CXX_STANDARD_REQUIRED)
  set (CMAKE_CXX_STANDARD_REQUIRED ${PREV_CMAKE_CXX_STANDARD_REQUIRED})
endif ()

if (PREV_CMAKE_CXX_EXTENSIONS)
  set (CMAKE_CXX_EXTENSIONS ${PREV_CMAKE_CXX_EXTENSIONS})
endif ()

unset (PREV_CMAKE_CXX_STANDARD)
unset (PREV_CMAKE_CXX_STANDARD_REQUIRED)
unset (PREV_CMAKE_CXX_EXTENSIONS)
unset (IMPORTING_LIBRARY_NAME CACHE)
unset (IMPORTING_LIBRARY CACHE)
unset (PREV_EXECUTABLE_OUTPUT_PATH)
endif ()
endmacro ()

# ----------------------------------------------------------------------------
# import a system global library that has been installed using the system's
# package manager.
macro (import_installed_library _name)

find_package (${_name} REQUIRED)

string (TOUPPER ${_name} name_upper)
#message ("found lib: ${_name}")
#message ("inc dir: ${${_name}_INCLUDE_DIR}")
#message ("inc dirs: ${${_name}_INCLUDE_DIRS}")
#message ("global defs: ${${_name}_GLOBAL_DEFINITIONS}")
#message ("inc dir: ${${name_upper}_INCLUDE_DIR}")
#message ("inc dirs: ${${name_upper}_INCLUDE_DIRS}")
#message ("global defs: ${${name_upper}_GLOBAL_DEFINITIONS}")

include_directories (${${_name}_INCLUDE_DIR} ${${_name}_INCLUDE_DIRS})
include_directories (${${name_upper}_INCLUDE_DIR} ${${name_upper}_INCLUDE_DIRS})

if (${${_name}_GLOBAL_DEFINITIONS})
  add_global_toolchain_definitions (${${_name}_GLOBAL_DEFINITIONS})
endif ()

if (${${name_upper}_GLOBAL_DEFINITIONS})
  add_global_toolchain_definitions (${${_name}_GLOBAL_DEFINITIONS})
endif ()

#message ("ARAVIS_FOUND : " ${aravis_FOUND})
#	message ("ARAVIS_INCLUDE_DIR : " ${aravis_INCLUDE_DIR})
#	message ("ARAVIS_LIBRARIES : " ${aravis_LIBRARIES})
#	message ("aravis_LIBRARY : " ${aravis_LIBRARY})
#	include_directories(${aravis_INCLUDE_DIR})
#	#link_directories(${ARAVIS_LIBRARY_DIRS})
#	target_link_libraries(${EXE_NAME} ${aravis_LIBRARIES})

endmacro ()

# ----------------------------------------------------------------------------
# unfortunately this doesn't work for the top-level project.
# the toolchain file (and thus this common file) will be processed
# after the first encounter of the project command.
# thus we need to do the "include_directories" for the top-level project
# explicitly.

macro (project name)

  # when the "project" is overridden, calling the original function
  # will not set PROJECT_NAME, PROJECT_VERSION and PROJECT_LANGUAGES
  # when cmake is invoked the very first time.
  set (options "")
  set (oneValueArgs VERSION)
  set (multiValueArgs LANGUAGES)
  cmake_parse_arguments (PROJECTTT "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  #message ("project version: ${PROJECTTT_VERSION}")
  #message ("project languages: ${PROJECTTT_LANGUAGES}")
  #message ("project unparsed args: ${PROJECTTT_UNPARSED_ARGUMENTS}")

  set (PROJECT_NAME "${name}")
  set (PROJECT_VERSION "${PROJECTTT_VERSION}")
  set (PROJECT_LANGUAGES "${PROJECTTT_LANGUAGES}")

  if (INSIDE_PROJECT_MACRO_OVERRIDE)

  else ()
    set (INSIDE_PROJECT_MACRO_OVERRIDE TRUE)

    if (LIBRARIES_DIR)
#message ("ADDING INC DIR ${LIBRARIES_DIR}")
      include_directories (${LIBRARIES_DIR})
    endif ()

    cmake_policy (SET CMP0048 NEW)
    _project (${name} ${ARGN})

    unset (INSIDE_PROJECT_MACRO_OVERRIDE)
  endif ()

  set (CMAKE_PROJECT_NAME ${PROJECT_NAME})
  set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

endmacro ()

# ----------------------------------------------------------------------------
# provide
#   add_target_library
#   add_target_executable
#
# as a replacement for the cmake built-in
#   add_library
#   add_executable
#
# in order to automatically append the dependencies to source generator
# targets.
#
# overriding the builtin add_library and add_exectuable is too troublesome.
# it doesn't work properly in some cases.
# for instance, in the overridden add_executable, calling _add_exectuable
# will not add the target.  doing an if (TARGET ... ) on the newly added
# target will return false.

macro (add_target_library target)

  add_library (${target} ${ARGN})

  # message ("target library add dependencies ${target} ${SOURCE_GENERATOR_TARGETS}")

  if (SOURCE_GENERATOR_TARGETS)
    add_dependencies (${target} ${SOURCE_GENERATOR_TARGETS})
  endif ()

endmacro ()

function (add_target_executable target)

 if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
 endif(COMMAND cmake_policy)

  # cmake 3.11 supports add_executable with an empty source list, so that
  # target_sources can be used on it later.
  if (${ARGC} GREATER 1)
    add_executable (${target} ${ARGN})
  else ()
    add_executable (${target} "")
  endif ()

  # message ("target executable add dependencies ${target} ${SOURCE_GENERATOR_TARGETS}")

  if (SOURCE_GENERATOR_TARGETS)
    add_dependencies (${target} ${SOURCE_GENERATOR_TARGETS})
  endif ()

endfunction ()

# ----------------------------------------------------------------------------

function (get_target_output_file output_var target)

get_target_property (type__ ${target} TYPE)

get_target_property (output_name__ ${target} OUTPUT_NAME)
get_target_property (binary_dir__ ${target} BINARY_DIR)

if (type__ STREQUAL "STATIC_LIBRARY")
  set (${output_var} "${binary_dir__}/${CMAKE_STATIC_LIBRARY_PREFIX}${output_name__}${CMAKE_STATIC_LIBRARY_SUFFIX}" PARENT_SCOPE)
elseif (type__ STREQUAL "SHARED_LIBRARY")
  set (${output_var} "${binary_dir__}/${CMAKE_SHARED_LIBRARY_PREFIX}${output_name__}${CMAKE_SHARED_LIBRARY_SUFFIX}" PARENT_SCOPE)
elseif (type__ STREQUAL "EXECUTABLE")
  set (${output_var} "${binary_dir__}/${output_name__}${CMAKE_EXECUTABLE_SUFFIX }" PARENT_SCOPE)
else ()
  message (FATAL_ERROR "unsupported type ${type__} of target ${target}")
endif()

endfunction()


# ----------------------------------------------------------------------------

macro (add_host_tool_executable target cxx_defines)

  if (CMAKE_CROSSCOMPILING)

    get_property (dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

    #message ("add_host_tool_exectuable ${target} dirs: ${dirs}")

    list (APPEND host_cxx_options "-std=c++17")
    list (APPEND host_cxx_options "-O2")
    list (APPEND host_cxx_options ${cxx_defines})

    foreach (dir ${dirs})
      list (APPEND host_cxx_options "-I${dir}")
    endforeach ()
    # the above also appends the current toolchain's directories,
    # which we don't want on the host.  so remove them again.
    list (REMOVE_ITEM host_cxx_options "-I${TOOLCHAIN_DIR}/include")
    list (REMOVE_ITEM host_cxx_options "-I${BOARD_DIR}/include")

    list (REMOVE_DUPLICATES host_cxx_options)

    #message ("add_host_tool_exectuable toolchain: ${TOOLCHAIN_DIR}")
    #message ("add_host_tool_exectuable ${target} dirs: ${host_cxx_options}")

    # we'd like to have all the host tools in the host_tool directory.  but
    # if that's the only output of the add_custom_command, cmake's dependency
    # checking won't work if we use that as a "DEPENDS ..." .. it has to be
    # a plain target name, like for add_executable.
    # as a work around, specify two outputs.  host_tool/${target} will be the
    # actual exectuable and ${target} will be some file somewhere in the
    # build directory just to make the dependencies work.
    add_custom_command (
      OUTPUT ${target} "${CMAKE_CURRENT_BINARY_DIR}/host_tool/${target}"
      COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/host_tool"
      COMMAND g++ ${host_cxx_options} -o "${CMAKE_CURRENT_BINARY_DIR}/host_tool/${target}" ${ARGN}
      COMMAND touch ${target}
      DEPENDS ${ARGN}
    )

  else ()

    set (PREV_CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/host_tool")

    foreach (i in ${CMAKE_CONFIGURATION_TYPES})
      string (TOUPPER ${i} i)
      set (PREV_CMAKE_RUNTIME_OUTPUT_DIRECTORY_${i} "${CMAKE_RUNTIME_OUTPUT_DIRECTORY_${i}}")
      set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_${i} "${CMAKE_CURRENT_BINARY_DIR}/host_tool")
    endforeach ()


    if (MSVC)
      set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_$<CONFIG>  "${CMAKE_CURRENT_BINARY_DIR}/host_tool")
    endif ()

    add_executable (${target} ${ARGN})
    target_compile_definitions (${target} PRIVATE ${cxx_defines})

    # when building a windows app, if "import_library" is used after the windows
    # app type has been set to "WINDOWS", it will try to build the host tool as
    # a GUI app, too (use WinMain instead of main ...).  make sure that we build
    # a console app.
    if (MSVC)
      target_link_options (${target} PRIVATE /SUBSYSTEM:CONSOLE)
    endif ()

    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PREV_CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    if ("${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" STREQUAL "")
      unset (CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    endif ()

    foreach (i in ${CMAKE_CONFIGURATION_TYPES})
      string (TOUPPER ${i} i)
      set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_${i} "${PREV_CMAKE_RUNTIME_OUTPUT_DIRECTORY_${i}}")
      if ("${CMAKE_RUNTIME_OUTPUT_DIRECTORY_${i}}" STREQUAL "")
        unset (CMAKE_RUNTIME_OUTPUT_DIRECTORY_${i})
      endif ()
    endforeach ()

  endif ()

endmacro ()

# ----------------------------------------------------------------------------

# this can be used by things like the BSP's toolchain additions to convert
# cmake build options to global defines/macros.

set (GLOBAL_TOOLCHAIN_DEFINITIONS "" CACHE INTERNAL "")

macro (add_global_toolchain_definitions _vars)
  set (GLOBAL_TOOLCHAIN_DEFINITIONS "${GLOBAL_TOOLCHAIN_DEFINITIONS} ${_vars}" CACHE INTERNAL "")
endmacro ()

# ----------------------------------------------------------------------------

# like target_link_libraries, but for a group of libraries to resolve
# cyclic dependencies.
macro (target_link_library_group target)
  target_link_libraries (${target}
  -Wl,--start-group ${ARGN} -Wl,--end-group)
endmacro ()

# ----------------------------------------------------------------------------

# like target_link_libraries, but force linking of all symbols, not only
# the one that are referenced.
macro (target_link_libraries_force target)
  target_link_libraries (${target}
  -Wl,--whole-archive ${ARGN} -Wl,--no-whole-archive)
endmacro ()

# ----------------------------------------------------------------------------
# cmake's builtin find_path and find_file do not find the path of
# c++ standard include directories.
# this function invokes the compiler and gets the path of the included
# file from the dependencies output that is generated by the compiler.

# in case of MSVC, find_file works OK when used from the "Visual Studio Command Promt"
# but it doesn't find anything when invoked from the regular command promt.
function (find_include_path _include_file _path_out_var)

  if (MSVC)

    # it seems there is no way to pass the source snippet to the compiler
    # over stdin.  have to write a temporary file.
    set (tmpfile "${CMAKE_CURRENT_BINARY_DIR}/toolchain/find_include.cpp")

    file (WRITE ${tmpfile} "\
	#include <${_include_file}>
    ")

    execute_process (
      COMMAND ${CMAKE_CXX_COMPILER} /Fl /showIncludes ${tmpfile}
      OUTPUT_VARIABLE tmpvar OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE execres
    )

    if (NOT tmpvar)
      message (FATAL_ERROR "Can't run compiler.  Perhaps not running with VS command environment?\n${execres}")
    endif ()

    # first line of the output contains the name of the file being compiled.
    # the second line contains the path of the include, like:
    #   find_include.cpp
    #   Note: including file: C:\Program Files (x86)\Micro ...

    # get the 2nd line
    foreach (i RANGE 1)
      string (FIND ${tmpvar} "\n" pos)
      string (SUBSTRING ${tmpvar} 0 ${pos} line)
      math (EXPR pos "${pos} + 1")
      string (SUBSTRING ${tmpvar} ${pos} -1 tmpvar)
    endforeach ()

    # use the second ": " as the delimiter before the actual include path
    foreach (i RANGE 1)
      string (FIND ${line} ": " pos)
      math (EXPR pos "${pos} + 2")
      string (SUBSTRING ${line} ${pos} -1 line)
    endforeach ()

    # with some locales, there are several leading spaces before
    # the real path string. trim it first.
    string (STRIP ${line} line)

    get_filename_component (line ${line} DIRECTORY)
    if (line)
      get_filename_component (line ${line} REALPATH)
      set (${_path_out_var} "${line}/" PARENT_SCOPE)
    else ()
      set (${_path_out_var} "${_path_out_var}-NOTFOUND" PARENT_SCOPE)
    endif ()

  else ()
    # assume GCC or compatible with unix environment
    execute_process (
      COMMAND echo "#include ${_include_file}"
      COMMAND ${CMAKE_CXX_COMPILER} -std=c++17 -E -P -M -xc++ -
      COMMAND grep "-m 1" -o -P "(?<=/).*"
      COMMAND cut -f1 -d "\\"
#      COMMAND awk -F= "{ printf(\"/\"); print; }"
#      COMMAND tr -d '\n'
      OUTPUT_VARIABLE tmpvar OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    set (${_path_out_var} "/${tmpvar}" PARENT_SCOPE)
  endif ()

endfunction ()

# ----------------------------------------------------------------------------

# to support 'find_libray' and pick up the correct multi-lib libraries based on
# the compiler options we need to ask the compiler for the library paths based
# on the compiler options.
# normally that would go into the CMAKE_SYSTEM_PREFIX_PATH variable.  however,
# the results of 'find_library' are usually cached.  if the compiler flags are
# changed after the initial configuration, the cached 'find_library' results
# will point to the wrong libraries.
# it is difficult to invalidate all cached variables.  instead, set
# CMAKE_SYSTEM_PREFIX_PATH to links in the build directory which will be rewritten
# here.
macro (gcc_library_paths_to_cmake_system_prefix_path)

set (new_cmake_cxx_flags "${CMAKE_CXX_FLAGS}")

if (NOT prev_cmake_cxx_lib_flags
    OR NOT ${prev_cmake_cxx_lib_flags} STREQUAL ${new_cmake_cxx_flags})

  set (prev_cmake_cxx_lib_flags ${new_cmake_cxx_flags} CACHE INTERNAL "" FORCE)
  separate_arguments (new_cmake_cxx_flags)

  execute_process (
    COMMAND ${CMAKE_CXX_COMPILER} ${new_cmake_cxx_flags} -print-search-dirs
    COMMAND grep libraries
    COMMAND cut "-c13-"
    COMMAND sed -e "s/:/;/g"
    RESULT_VARIABLE lib_search_result
    OUTPUT_VARIABLE lib_search_dirs
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  if (lib_search_result)
    message (FATAL_ERROR "can't get compiler library paths")
  endif ()

  foreach (dir ${lib_search_dirs})
    get_filename_component (dir ${dir} ABSOLUTE)
    list (APPEND lib_search_dirs_ ${dir})
  endforeach ()

  list (REMOVE_DUPLICATES lib_search_dirs_)
  set (lib_search_dirs ${lib_search_dirs_})
  unset (lib_search_dirs_)

#  foreach (dir ${lib_search_dirs})
#    message ("lib search dir: ${dir}")
#  endforeach ()

  execute_process (COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/syslibs")
  execute_process (COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/syslibs")

  set (link_file_num "0")
  unset (all_dirlinks)

  foreach (dir ${lib_search_dirs})
    set (dirlink "${CMAKE_CURRENT_BINARY_DIR}/syslibs/link${link_file_num}")
    execute_process (COMMAND ${CMAKE_COMMAND} -E create_symlink ${dir} ${dirlink})
    list (APPEND all_dirlinks ${dirlink})
    math (EXPR link_file_num "${link_file_num}+1")
  endforeach ()

#  foreach (dir ${all_dirlinks})
#    message ("linked lib search dir: ${dir}")
#  endforeach ()

  set (CMAKE_SYSTEM_PREFIX_PATH ${all_dirlinks} CACHE INTERNAL "" FORCE)
endif ()

endmacro ()

# ----------------------------------------------------------------------------

get_filename_component (TOOLCHAIN_DIR ${CMAKE_TOOLCHAIN_FILE} DIRECTORY)
get_filename_component (TOOLCHAIN_DIR ${TOOLCHAIN_DIR} ABSOLUTE CACHE)
#mark_as_internal (TOOLCHAIN_DIR)

if (NOT CMAKE_BOARD_DIR OR CMAKE_BOARD_DIR STREQUAL "")

else ()
  get_filename_component (BOARD_DIR ${CMAKE_BOARD_DIR} ABSOLUTE CACHE)
#  mark_as_internal (BOARD_DIR)
endif ()

if (NOT CMAKE_LIBRARIES_DIR OR CMAKE_LIBRARIES_DIR STREQUAL "")
else ()
  get_filename_component (LIBRARIES_DIR ${CMAKE_LIBRARIES_DIR} ABSOLUTE CACHE)
#  mark_as_internal (BOARD_DIR)
endif ()

include (${CMAKE_CURRENT_SOURCE_DIR}/defaults.cmake OPTIONAL)

# ----------------------------------------------------------------------------

# override include paths have to be specified at the very beginning
# use cmake's facility for that or else the files in those directories will
# not be dependency tracked.
include_directories (BEFORE ${TOOLCHAIN_DIR}/include)

if (BOARD_DIR)
  include_directories (BEFORE ${BOARD_DIR}/include)
endif ()


# ----------------------------------------------------------------------------
# copy all header files that have the target property PUBLIC_HEADER set
# to the specified output directory.

function (copy_public_includes target output_dir)

  # get all public header files of the target
  get_target_property (TARGETS_PUBLIC_HEADERS_ ${target} PUBLIC_HEADER)
  foreach (file ${TARGETS_PUBLIC_HEADERS_})
    if (IS_ABSOLUTE ${file})
	  list (APPEND TARGETS_PUBLIC_HEADERS ${file})
	else ()
	  list (APPEND TARGETS_PUBLIC_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/${file}")
	endif ()
  endforeach ()

  if (TARGETS_PUBLIC_HEADERS)
    add_custom_command (TARGET ${target} POST_BUILD
	  COMMAND "${CMAKE_COMMAND}" -E make_directory ${output_dir}
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${TARGETS_PUBLIC_HEADERS} ${output_dir}
    )
  endif ()

endfunction ()
