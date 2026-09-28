# adding options
# http://stackoverflow.com/questions/6787371/how-do-i-specify-build-options-for-cmake-based-projects

# toolchain files
# http://www.vtk.org/Wiki/CMake_Cross_Compiling

set (CMAKE_SYSTEM_NAME Generic)
set (CMAKE_SYSTEM_VERSION 1)
set (CMAKE_SYSTEM_PROCESSOR rl78)
set (CMAKE_CROSSCOMPILING TRUE)

set (CMAKE_C_COMPILER /usr/local/bin/rl78-elf-gcc CACHE FILEPATH "target_c" FORCE)
set (CMAKE_CXX_COMPILER /usr/local/bin/rl78-elf-g++ CACHE FILEPATH "target_cxx" FORCE)

# have to override ar,nm and ranlib to use the gcc wrapper for LTO support.
# force it into the cmake cache or else it will try lookup them up later and
# get it wrong.
set (CMAKE_AR /usr/local/bin/rl78-elf-gcc-ar CACHE FILEPATH "target_ar" FORCE)
set (CMAKE_NM /usr/local/bin/rl78-elf-gcc-nm CACHE FILEPATH "target_nm" FORCE)
set (CMAKE_RANLIB /usr/local/bin/rl78-elf-gcc-ranlib CACHE FILEPATH "target_ranlib" FORCE)

# for some reason, can't enable the ASM language here.  it makes cmake eat
# all memory and eventually crash.  looks like an infinite loop somewhere.
#enable_language(ASM)

# search for programs in the build host directories
set (CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set (CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set (CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# ----------------------------------------------------------------------------

# the directory of the toolchain file is used to lookup other toolchain
# related files like linker script.

if (NOT CMAKE_TOOLCHAIN_FILE OR CMAKE_TOOLCHAIN_FILE STREQUAL "")

# don't add anything funny if the toolchain file is not set.
# this happens when cmake runs the initial "is compiler compiler working"
# checks.

else ()

include (${CMAKE_CURRENT_LIST_DIR}/../common.cmake)


# if LTO is used, we can't use "add_compile_options" because these will
# not be added to the final linker options, which is needed for the whole
# thing to work.
# https://cmake.org/pipermail/cmake-developers/2014-June/010623.html

option (TARGET_LTO "Link time optimization (LTO)" ON)
option (TARGET_FWEB "-fweb" OFF)
option (TARGET_FPEEL_LOOPS "-fpeel-loops" OFF)
option (TARGET_RELAX "Linker relaxation" OFF)
option (TARGET_GC_SECTIONS "Linker GC sections" ON)
option (TARGET_DATA_SECTIONS "Separate data sections" ON)
option (TARGET_FUNCTION_SECTIONS "Separate function sections" OFF)
option (TARGET_SAVE_TEMPS "Save intermediate compiler output files" OFF)
option (TARGET_VERBOSE_LINKER "Verbose linking" OFF)
option (TARGET_CXX_EXCEPTIONS "C++ Exceptions" ON)
option (TARGET_C_EXCEPTIONS "C Exceptions" OFF)
option (TARGET_CXX_RTTI "C++ RTTI" ON)
option (TARGET_DISABLE_GLOBAL_DTORS "Disable global destructor code" ON)
option (TARGET_ENABLE_ALL_WARNINGS "Enable all warnings" ON)
option (TARGET_FULL_STATIC "Full static linking" ON)
set (TARGET_OPTIMIZE "-O2" CACHE STRING "Optimization flags")
option (TARGET_STRIP_SYMBOLS "Strip symbols" OFF)
option (TARGET_DEBUG_INFO "Incude Debug Info" OFF)
option (TARGET_DISABLE_ASSERT "Disable standard assert (NDEBUG)" OFF)

set (TARGET_CPU "g10" CACHE STRING "CPU")
set_property (CACHE TARGET_CPU PROPERTY STRINGS g10 g13 g14)

set (TARGET_ROM_SIZE "1024*1" CACHE STRING "ROM size in bytes")
set (TARGET_RAM_SIZE "128" CACHE STRING "RAM size in bytes")
set (TARGET_STACK_SIZE "64" CACHE STRING "Stack size in bytes")

if (${TARGET_CPU} STREQUAL "g10")
  set (TARGET_ROM_MIRROR_ADDR "0xF8000")
else ()
  set (TARGET_ROM_MIRROR_ADDR "0xF0000")
endif ()


# if LTO is used it's better to disable function-sections as it
# results in smaller code.

if (TARGET_ENABLE_ALL_WARNINGS)
  set (OPT_WALL "-Wall")
endif ()

if (TARGET_DISABLE_ASSERT)
  set (OPT_NDEBUG "-DNDEBUG")
endif ()

if (TARGET_LTO)
  set (OPT_LTO "-flto=${BUILD_PROCESSOR_COUNT} -flto-compression-level=0")
endif ()

if (TARGET_FWEB)
  set (OPT_FWEB "-fweb")
endif ()

if (TARGET_RELAX)
  set (OPT_RELAX "-mrelax")
endif ()

if (TARGET_FPEEL_LOOPS)
  set (OPT_FPEEL_LOOPS "-fpeel-loops")
endif ()

if (TARGET_GC_SECTIONS)
  set (OPT_GC_SECTIONS "-Wl,--gc-sections")
endif ()

if (TARGET_DATA_SECTIONS)
  set (OPT_DATA_SECTIONS "-fdata-sections")
endif ()

if (TARGET_FUNCTION_SECTIONS)
  set (OPT_FUNCTION_SECTIONS "-ffunction-sections")
endif ()


if (TARGET_SAVE_TEMPS)
  set (OPT_SAVE_TEMPS "-save-temps")
else ()
  set (OPT_SAVE_TEMPS "-pipe")
endif ()

if (TARGET_VERBOSE_LINKER)
  set (LINKER_VERBOSE "-Wl,-verbose")
endif ()

if (TARGET_CXX_EXCEPTIONS)
  set (OPT_CXX_EXCEPTIONS "-fexceptions")
else ()
  set (OPT_CXX_EXCEPTIONS "-fno-exceptions")
endif ()

if (TARGET_C_EXCEPTIONS)
  set (OPT_C_EXCEPTIONS "-fexceptions")
else ()
  set (OPT_C_EXCEPTIONS "-fno-exceptions")
endif ()

if (TARGET_CXX_RTTI)
  set (OPT_CXX_NORTTI "")
else ()
  set (OPT_CXX_NORTTI "-fno-rtti")
endif ()

if (TARGET_DISABLE_GLOBAL_DTORS)
  set (OPT_DISABLE_GLOBAL_DTORS "-DDISABLE_GLOBAL_DTORS")
  file (WRITE ${CMAKE_CURRENT_BINARY_DIR}/global_dtors.ld "")

else ()
  file (WRITE ${CMAKE_CURRENT_BINARY_DIR}/global_dtors.ld "\
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array ))
    PROVIDE_HIDDEN (__fini_array_end = .);
  }  > rom_mirror AT> rom

  .dtors :
  {
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
  }  > rom_mirror AT> rom")

endif ()

file (WRITE ${CMAKE_CURRENT_BINARY_DIR}/memory_config.ld "\
  _rl78_rom_size = ${TARGET_ROM_SIZE};
  _rl78_ram_size = ${TARGET_RAM_SIZE};
  _rl78_stack_size = ${TARGET_STACK_SIZE};
  _rl78_rom_mirror_start = ${TARGET_ROM_MIRROR_ADDR};
")


set (TARGET_LINKER_SCRIPT ${TOOLCHAIN_DIR}/linker_script.ld)
mark_as_internal (TARGET_LINKER_SCRIPT)

set (TARGET_LINKER_SCRIPT_DEPS
  ${CMAKE_CURRENT_BINARY_DIR}/global_dtors.ld
  ${CMAKE_CURRENT_BINARY_DIR}/memory_config.ld
  ${TARGET_LINKER_SCRIPT}
)
mark_as_internal (TARGET_LINKER_SCRIPT_DEPS)

set (LINKER_STATIC "-Wl,-Bstatic")

# default options

# GCC 7
# -m64bit-doubles is documented but not actually implemented.
# https://gcc.gnu.org/bugzilla/show_bug.cgi?id=71340
#set (TARGET_OPTIONS "-mcpu=${TARGET_CPU}")

if (${TARGET_CPU} STREQUAL "g13")
  set (TARGET_OPTIONS "-msave-mduc-in-interrupts")
endif ()


# GCC 6
#set (TARGET_OPTIONS "-mcpu=${TARGET_CPU}")


# GCC 5
if (${TARGET_CPU} STREQUAL "g10")
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -mg10")
elseif (${TARGET_CPU} STREQUAL "g13")
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -mg13")
elseif (${TARGET_CPU} STREQUAL "g14")
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -mg14")
endif ()

set (C_LANG_OPTIONS "-Werror=return-type ${OPT_WALL}")
set (CXX_LANG_OPTIONS "-Werror=return-type ${OPT_WALL}")

if (TARGET_FULL_STATIC)
  set (LINKER_STATIC "-Wl,-Bstatic")
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -static")
endif ()

if (TARGET_STRIP_SYMBOLS)
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -s")
endif ()

if (TARGET_DEBUG_INFO)
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -g")
endif ()

macro (rebuild_build_flags)

# https://gcc.gnu.org/ml/libstdc++/2013-10/msg00245.html
# https://answers.launchpad.net/gcc-arm-embedded/+question/238327
set (LIBSTDCPP_DEFINES "-D_GLIBCXX_USE_C99=1 -D_GLIBCXX_USE_C99_CHECK=1 -D_GLIBCXX_USE_C99_DYNAMIC=1 -D_GLIBCXX_USE_C99_LONG_LONG_DYNAMIC=1")

set (CMAKE_C_FLAGS "\
${LIBSTDCPP_DEFINES} \
${HAVE_BOARD_CONFIG} \
${INCLUDE_OVERRIDES} \
${TARGET_OPTIMIZE} \
${OPT_C_EXCEPTIONS} \
${C_LANG_OPTIONS} \
${TARGET_OPTIONS} \
${ASM_OPTIONS} \
${OPT_NDEBUG} \
${OPT_FUNCTION_SECTIONS} \
${OPT_DATA_SECTIONS} \
${OPT_SAVE_TEMPS} \
${OPT_LTO} \
${OPT_FWEB} \
${OPT_FPEEL_LOOPS} \
${OPT_RELAX} \
${OPT_DISABLE_GLOBAL_DTORS} \
" CACHE STRING "target_c_flags" FORCE)

set (CMAKE_CXX_FLAGS "\
${LIBSTDCPP_DEFINES} \
${HAVE_BOARD_CONFIG} \
${INCLUDE_OVERRIDES} \
${TARGET_OPTIMIZE} \
${OPT_CXX_EXCEPTIONS} \
${OPT_CXX_NORTTI} \
${CXX_LANG_OPTIONS} \
${TARGET_OPTIONS} \
${ASM_OPTIONS} \
${OPT_NDEBUG} \
${OPT_FUNCTION_SECTIONS} \
${OPT_DATA_SECTIONS} \
${OPT_SAVE_TEMPS} \
${OPT_LTO} \
${OPT_FWEB} \
${OPT_FPEEL_LOOPS} \
${OPT_RELAX} \
${OPT_DISABLE_GLOBAL_DTORS} \
" CACHE STRING "target_cxx_flags" FORCE)

# note: cmake uses gcc as the assembler for .S files.
set (CMAKE_ASM_FLAGS ${CMAKE_C_FLAGS} CACHE STRING "target_asm_flags" FORCE)

set (CMAKE_EXE_LINKER_FLAGS "\
${OPT_GC_SECTIONS} -nostartfiles -nodefaultlibs \
${ASM_OPTIONS} ${LINKER_VERBOSE} ${LINKER_STATIC} \
-T '${TARGET_LINKER_SCRIPT}' \
" CACHE STRING "target_ld_flags" FORCE)

set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_CXX_FLAGS}" CACHE STRING "target_ld_flags_shared" FORCE)

endmacro ()

rebuild_build_flags ()

# ----------------------------------------------------------------------------
macro (add_additional_executable_targets _var)

get_filename_component (TARGET_EXE_FILENAME_WE ${_var} NAME_WE)

add_custom_command (TARGET ${_var}
  POST_BUILD COMMAND rl78-elf-size ${_var})

add_custom_target ("${TARGET_EXE_FILENAME_WE}.mot"
  COMMAND ${CMAKE_OBJCOPY} ${TARGET_OBJCOPY_OPTIONS} --srec-forceS3 --srec-len 32 -O srec ${_var} ${TARGET_EXE_FILENAME_WE}.mot
  DEPENDS ${_var}
)

# http://stackoverflow.com/questions/19019199/raw-binary-file-generated-by-objcopy-is-too-big
add_custom_target ("${TARGET_EXE_FILENAME_WE}.bin"
  COMMAND ${CMAKE_OBJCOPY} ${TARGET_OBJCOPY_OPTIONS} -O binary ${_var} ${TARGET_EXE_FILENAME_WE}.bin
  DEPENDS ${_var}
)


if (EXISTS "${BOARD_DIR}/toolchain_additions.cmake")

include ("${BOARD_DIR}/toolchain_additions.cmake")

endif ()

endmacro()

endif ()
