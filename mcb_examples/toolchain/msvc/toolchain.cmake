# ----------------------------------------------------------------------------

set (CMAKE_SYSTEM_NAME Windows)
set (CMAKE_CROSSCOMPILING FALSE)


if (NOT CMAKE_TOOLCHAIN_FILE OR CMAKE_TOOLCHAIN_FILE STREQUAL "")

# don't add anything funny if the toolchain file is not set.
# this happens when cmake runs the initial "is compiler compiler working"
# checks.

# it seems cmake-gui has a problem.. it always resets (clears) the
# CMAKE_TOOLCHAIN_FILE variable after the initial invocation.
# as a workaround, remember it in another variable GOT_CMAKE_TOOLCHAIN_FILE.

else ()
  # message ("got toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
  set (GOT_CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_FILE}" INTERNAL)
endif ()


if (GOT_CMAKE_TOOLCHAIN_FILE AND NOT GOT_CMAKE_TOOLCHAIN_FILE STREQUAL "")

if (NOT CMAKE_TOOLCHAIN_FILE OR CMAKE_TOOLCHAIN_FILE STREQUAL "")
  set (CMAKE_TOOLCHAIN_FILE ${GOT_CMAKE_TOOLCHAIN_FILE})
endif ()

include (${CMAKE_CURRENT_LIST_DIR}/../common.cmake)

if (NOT TARGET_ENDIAN)
  set (TARGET_ENDIAN "little" CACHE STRING "Endian")
endif ()

# TARGET_BINLIB_DIR is used for selecting pre-build binary libraries
set (TARGET_BINLIB_DIR "${CMAKE_GENERATOR_PLATFORM}/$<CONFIG>")

# these do not work properly.
# have to specify in the application's cmake project file for now.
#[[
option (TARGET_LTO "Link time optimization" ON)
option (TARGET_COMDAT_FOLDING "COMDAT folding" ON)
option (TARGET_ENABLE_LARGEADDRESS "Enable large address awareness" ON)

# subsystem can't be set here because during the "try_compile" compiler test
# this is always used and then it will fail to link.
# set it in the application's cmakelists

# set (TARGET_SUBSYSTEM "WINDOWS" CACHE STRING "Subsystem type")
#set_property (CACHE TARGET_SUBSYSTEM PROPERTY STRINGS WINDOWS CONSOLE NATIVE POSIX)

option (TARGET_PARALLEL_BUILD "Parallel build" ON)

set (TARGET_DEBUG_INFO "DEBUG" CACHE STRING "Debug info type")
set_property (CACHE TARGET_DEBUG_INFO PROPERTY STRINGS DEBUG DEBUG_FAST NONE)

option (TARGET_MINIMAL_REBUILD "Minimal rebuild" ON)

#]]


macro (rebuild_build_flags)

# on msvc we always use configuration names such as debug release, and so on.
# CMAKE_C_FLAGS, CMAKE_CXX_FLAGS, CMAKE_EXE_LINKER_FLAGS, ... contain the common
# flags for all configurations.

set (LINKER_OPT_FLAGS "")
set (C_CXX_OPT_FLAGS "")

if (TARGET_LTO)
  set (CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
else ()
  unset (CMAKE_INTERPROCEDURAL_OPTIMIZATION)
endif ()

# set (LINKER_OPT_FLAGS "${LINKER_OPT_FLAGS} /SUBSYSTEM:${TARGET_SUBSYSTEM}")

if (TARGET_COMDAT_FOLDING)
  set (LINKER_OPT_FLAGS "${LINKER_OPT_FLAGS} /OPT:ICF")
endif ()

if (TARGET_DEBUG_INFO STREQUAL "DEBUG")
  set (LINKER_OPT_FLAGS "${LINKER_OPT_FLAGS} /DEBUG")
elseif (TARGET_DEBUG_INFO STREQUAL "DEBUG_FAST")
  set (LINKER_OPT_FLAGS "${LINKER_OPT_FLAGS} /DEBUG:FASTLINK")
endif ()

if (TARGET_ENABLE_LARGEADDRESS)
  set (LINKER_OPT_FLAGS "${LINKER_OPT_FLAGS} /LARGEADDRESSAWARE")
endif ()


if (TARGET_PARALLEL_BUILD)
  set (C_CXX_OPT_FLAGS "${C_CXX_OPT_FLAGS} /MP")
endif ()

if (TARGET_MINIMAL_REBUILD)
  set (C_CXX_OPT_FLAGS "${C_CXX_OPT_FLAGS} /Gm")
endif ()

if (NOT PREV_CMAKE_CXX_FLAGS)
  set (PREV_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
endif ()

if (NOT PREV_CMAKE_C_FLAGS)
  set (PREV_CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
endif ()

if (NOT PREV_CMAKE_EXE_LINKER_FLAGS)
  set (PREV_CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS})
endif ()

set (CMAKE_CXX_FLAGS ${PREV_CMAKE_CXX_FLAGS} ${C_CXX_OPT_FLAGS})
set (CMAKE_C_FLAGS ${PREV_CMAKE_C_FLAGS} ${C_CXX_OPT_FLAGS})
set (CMAKE_EXE_LINKER_FLAGS ${PREV_CMAKE_EXE_LINKER_FLAGS} ${LINKER_OPT_FLAGS})
endmacro ()

# FIXME: somehow this is not really working.
#        it seems that if CMAKE_CXX_FLAGS are not set, cmake will
#        set some default options from _$<CONFIG>.  otherwise it will
#        override the _$<CONFIG> settings with CMAKE_CXX_FLAGS ...

# rebuild_build_flags ()

endif ()
