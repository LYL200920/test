
# FIXME: MCB-100
# somehow the cmake dependency scanning doesn't work properly in this case.
# user code includes "utils/pp_for_each", which in turn includes the generated
# header file.
# it seems cmake detects dependencies on generated header files only if those
# header files are generated straight into the build directory and if they
# are included as such, without any indirection (through other includes).

# for everything else, we have to specify an explicit target dependency, which
# is a bit annoying, because we'd like to have this happen automatically.

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED 14)
set (CMAKE_CXX_EXTENSIONS OFF)

add_host_tool_executable (gen_pp_for_each "" ${CMAKE_CURRENT_LIST_DIR}/pp_for_each.cpp)
add_host_tool_executable (gen_pp_args "" ${CMAKE_CURRENT_LIST_DIR}/pp_args.cpp)
add_host_tool_executable (gen_pp_arith "" ${CMAKE_CURRENT_LIST_DIR}/pp_arith.cpp)

if (MSVC)
  set (PP_FOR_EACH_MAX_COUNT 125)
  set (PP_ARGS_MAX_COUNT 62)
  set (PP_ARITH_MAX_COUNT 125)
else ()
  set (PP_FOR_EACH_MAX_COUNT 256)
  set (PP_ARGS_MAX_COUNT 256)
  set (PP_ARITH_MAX_COUNT 256)
endif ()


add_custom_command (
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_for_each.hpp"
  COMMAND "${CMAKE_CURRENT_BINARY_DIR}/host_tool/gen_pp_for_each" ${PP_FOR_EACH_MAX_COUNT} > ${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_for_each.hpp
  DEPENDS gen_pp_for_each
)

add_custom_command (
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_args.hpp"
  COMMAND "${CMAKE_CURRENT_BINARY_DIR}/host_tool/gen_pp_args" ${PP_ARGS_MAX_COUNT} > ${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_args.hpp
  DEPENDS gen_pp_args
)

add_custom_command (
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_arith.hpp"
  COMMAND "${CMAKE_CURRENT_BINARY_DIR}/host_tool/gen_pp_arith" ${PP_ARITH_MAX_COUNT} > ${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_arith.hpp
  DEPENDS gen_pp_arith
)

# notice that we are in the scope of the project that includes/uses this library
include_directories ( ${CMAKE_CURRENT_BINARY_DIR} )

SET_SOURCE_FILES_PROPERTIES (${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_for_each.hpp PROPERTIES GENERATED 1)
SET_SOURCE_FILES_PROPERTIES (${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_args.hpp PROPERTIES GENERATED 1)
SET_SOURCE_FILES_PROPERTIES (${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_arith.hpp PROPERTIES GENERATED 1)


add_custom_target (generate_utils_pp ALL DEPENDS
 "${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_for_each.hpp"
 "${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_args.hpp"
 "${CMAKE_CURRENT_BINARY_DIR}/gen_includes_utils_pp_arith.hpp"
)

# dependencies will be automatically added by toolchain's import_library,
# add_target_library and add_target_executable commands, but not for the
# current library.
add_dependencies (utils generate_utils_pp)

list (APPEND SOURCE_GENERATOR_TARGETS generate_utils_pp)

