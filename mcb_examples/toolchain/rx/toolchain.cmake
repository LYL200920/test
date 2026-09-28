# adding options
# http://stackoverflow.com/questions/6787371/how-do-i-specify-build-options-for-cmake-based-projects

# toolchain files
# http://www.vtk.org/Wiki/CMake_Cross_Compiling

set (CMAKE_SYSTEM_NAME Generic)
set (CMAKE_SUBSYSTEM_NAME EmbeddedMCU)
set (CMAKE_SYSTEM_VERSION 1)
set (CMAKE_SYSTEM_PROCESSOR rx)
set (CMAKE_CROSSCOMPILING TRUE)

# the toolchain version number must match the number in the build_install.sh
# script.
set (TOOLCHAIN_VERSION "gcc8-190509")
set (PREFIX_PATH "/opt/jz/${TOOLCHAIN_VERSION}/rx-elf")


set (CMAKE_C_COMPILER ${PREFIX_PATH}/bin/rx-elf-gcc CACHE FILEPATH "target_c" FORCE)
set (CMAKE_CXX_COMPILER ${PREFIX_PATH}/bin/rx-elf-g++ CACHE FILEPATH "target_cxx" FORCE)

# have to override ar,nm and ranlib to use the gcc wrapper for LTO support.
# force it into the cmake cache or else it will try lookup them up later and
# get it wrong.
set (CMAKE_AR ${PREFIX_PATH}/bin/rx-elf-gcc-ar CACHE FILEPATH "target_ar" FORCE)
set (CMAKE_NM ${PREFIX_PATH}/bin/rx-elf-gcc-nm CACHE FILEPATH "target_nm" FORCE)
set (CMAKE_RANLIB ${PREFIX_PATH}/bin/rx-elf-gcc-ranlib CACHE FILEPATH "target_ranlib" FORCE)

set (CMAKE_SIZE ${PREFIX_PATH}/bin/rx-elf-size)

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

# these are expected to be defined by the board's toolchain config.
# they are used to generate the linker script.
#set (TARGET_ROM_END "0xFFFFFFFF" CACHE STRING "ROM end address")
#set (TARGET_ROM_SIZE "1024*256" CACHE STRING "ROM size in bytes")
#set (TARGET_RAM_START "0x00000100" CACHE STRING "RAM start address")
#set (TARGET_RAM_SIZE "1024*128-256" CACHE STRING "RAM size in bytes")
#set (TARGET_ISTACK_SIZE "1024" CACHE STRING "istack size in bytes")
#set (TARGET_USTACK_SIZE "1024*2" CACHE STRING "ustack size in bytes")

include ("${BOARD_DIR}/toolchain_options.cmake")

set (TARGET_ENDIAN "little" CACHE STRING "Endian")
set_property (CACHE TARGET_ENDIAN PROPERTY STRINGS little big)

set (TARGET_CPU "RX600" CACHE STRING "CPU Type")
set_property (CACHE TARGET_CPU PROPERTY STRINGS RX600 RX200 RX610 RXv2)

# if LTO is used it's better to disable function-sections as it
# results in smaller code.

if (TARGET_ENABLE_ALL_WARNINGS)
  set (OPT_WALL "-Wall")
endif ()

if (TARGET_DISABLE_ASSERT)
  set (OPT_NDEBUG "-DNDEBUG")
endif ()

if (TARGET_LTO)
  set (OPT_LTO "-flto=${BUILD_PROCESSOR_COUNT} -fdevirtualize-at-ltrans")
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

set (OPT_DISABLE_GLOBAL_DTORS "" CACHE INTERNAL "")

macro (board_toolchain_add_linkerscript _linkerscript_dummy_sourcefile)

if (TARGET_DISABLE_GLOBAL_DTORS)
  set (OPT_DISABLE_GLOBAL_DTORS "-DDISABLE_GLOBAL_DTORS" CACHE INTERNAL "")
  file (WRITE ${CMAKE_CURRENT_BINARY_DIR}/global_dtors.ld "")

else ()
  file (WRITE ${CMAKE_CURRENT_BINARY_DIR}/global_dtors.ld "\
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array ))
    PROVIDE_HIDDEN (__fini_array_end = .);
  }

  .dtors :
  {
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
  }")

endif ()

file (WRITE ${CMAKE_CURRENT_BINARY_DIR}/memory_config.ld "\
  _rx_rom_end = ${TARGET_ROM_END};
  _rx_rom_size = ${TARGET_ROM_SIZE};
  _rx_ram_start = ${TARGET_RAM_START};
  _rx_ram_size = ${TARGET_RAM_SIZE};
  _rx_istack_size = ${TARGET_ISTACK_SIZE};
  _rx_ustack_size = ${TARGET_USTACK_SIZE};
")


set (TARGET_LINKER_SCRIPT ${TOOLCHAIN_DIR}/linker_script.ld)
mark_as_internal (TARGET_LINKER_SCRIPT)

set (TARGET_LINKER_SCRIPT_DEPS
  ${CMAKE_CURRENT_BINARY_DIR}/global_dtors.ld
  ${CMAKE_CURRENT_BINARY_DIR}/memory_config.ld
  ${TARGET_LINKER_SCRIPT}
)
mark_as_internal (TARGET_LINKER_SCRIPT_DEPS)

# the linker script to be used is defined by the toolchain file.
# here we just add the dependency to get recompiled if the linker script changes.
# see also
# https://cmake.org/pipermail/cmake/2010-May/037206.html

set_source_files_properties (
  ${_linkerscript_dummy_sourcefile} OBJECT_DEPENDS "${TARGET_LINKER_SCRIPT_DEPS}"
)

endmacro ()


set (LINKER_STATIC "-Wl,-Bstatic")

# default options
set (TARGET_OPTIONS "-m64bit-doubles -msave-acc-in-interrupts -Wa,--muse-conventional-section-names")

if (${TARGET_CPU} STREQUAL "RX600")
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -mcpu=RX600")
elseif (${TARGET_CPU} STREQUAL "RX200")
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -mcpu=RX200")
elseif (${TARGET_CPU} STREQUAL "RX610")
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -mcpu=RX610")
elseif (${TARGET_CPU} STREQUAL "RXv2")
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -mcpu=RX600")
endif ()

if(${TARGET_ENDIAN} STREQUAL "big")
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -mbig-endian-data -Wl,--oformat=elf32-rx-be -Wa,-mbig-endian")
#  set (TARGET_OBJCOPY_OPTIONS "--input-target=elf32-rx-be-ns")
elseif (${TARGET_ENDIAN} STREQUAL "little")
  set (TARGET_OPTIONS "${TARGET_OPTIONS} -mlittle-endian-data -Wl,--oformat=elf32-rx-le -Wa,-mlittle-endian")
  set (TARGET_OBJCOPY_OPTIONS "")
else ()
  message (FATAL_ERROR "undefined target endian setting")
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
${GLOBAL_TOOLCHAIN_DEFINITIONS} \
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
${GLOBAL_TOOLCHAIN_DEFINITIONS} \
" CACHE STRING "target_cxx_flags" FORCE)

# note: cmake uses gcc as the assembler for .S files.
set (CMAKE_ASM_FLAGS ${CMAKE_C_FLAGS} CACHE STRING "target_asm_flags" FORCE)

if (TARGET_LINKER_SCRIPT)
  # if a sub-module builds an executable that uses the generated linker script pieces
  # it needs to be found in the board build directory.  hence add the search path here.
  set (USE_TARGET_LINKER_SCRIPT "-T '${TARGET_LINKER_SCRIPT}' -L '${CMAKE_CURRENT_BINARY_DIR}'")
else ()
  unset (USE_TARGET_LINKER_SCRIPT)
endif ()

set (CMAKE_EXE_LINKER_FLAGS "\
${OPT_GC_SECTIONS} -nostartfiles -nodefaultlibs \
${ASM_OPTIONS} ${LINKER_VERBOSE} ${LINKER_STATIC} \
${USE_TARGET_LINKER_SCRIPT} \
" CACHE STRING "target_ld_flags" FORCE)

set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_CXX_FLAGS}" CACHE STRING "target_ld_flags_shared" FORCE)

gcc_library_paths_to_cmake_system_prefix_path ()

endmacro ()

rebuild_build_flags ()

# ----------------------------------------------------------------------------
macro (add_additional_executable_targets _var)

get_filename_component (TARGET_EXE_FILENAME_WE ${_var} NAME_WE)

add_custom_command (TARGET ${_var}
  POST_BUILD COMMAND ${CMAKE_SIZE} ${_var})

# use one indirection to avoid overwriting the .mot and .bin files, which would
# happen if we used only "add_custom_target".  one drawback of this
# is that the .mot and .bin files are byproducts and the .stamp files are the
# main output files.  make clean will thus not delete the .mot and .bin files.
add_custom_command (
  OUTPUT "${TARGET_EXE_FILENAME_WE}.mot.stamp"
  COMMAND ${CMAKE_COMMAND} -E touch "${TARGET_EXE_FILENAME_WE}.mot.stamp"
  COMMAND ${CMAKE_OBJCOPY} ${TARGET_OBJCOPY_OPTIONS} --srec-forceS3 --srec-len 32 -O srec ${_var} "${TARGET_EXE_FILENAME_WE}.mot"
  COMMENT "Generating ${TARGET_EXE_FILENAME_WE}.mot"
  DEPENDS ${_var}
)

add_custom_target ("${TARGET_EXE_FILENAME_WE}.mot"
  DEPENDS "${TARGET_EXE_FILENAME_WE}.mot.stamp"
)

# http://stackoverflow.com/questions/19019199/raw-binary-file-generated-by-objcopy-is-too-big
add_custom_command (
  OUTPUT "${TARGET_EXE_FILENAME_WE}.bin.stamp"
  COMMAND ${CMAKE_COMMAND} -E touch "${TARGET_EXE_FILENAME_WE}.bin.stamp"
  COMMAND ${CMAKE_OBJCOPY} ${TARGET_OBJCOPY_OPTIONS} -O binary "${_var}" "${TARGET_EXE_FILENAME_WE}.bin"
  COMMENT "Generating ${TARGET_EXE_FILENAME_WE}.bin"
  DEPENDS ${_var}
)

add_custom_target ("${TARGET_EXE_FILENAME_WE}.bin"
  DEPENDS "${TARGET_EXE_FILENAME_WE}.bin.stamp"
)


# this could be an alternative, but for some reason, the dependency on the
# ELF target does not work.  so if the ELF is rewritten, the BIN and MOT will
# not be remade.
#set (CMAKE_ELF_TO_BIN_LINK_EXECUTABLE "${CMAKE_OBJCOPY} ${TARGET_OBJCOPY_OPTIONS} -O binary ${_var} ${TARGET_EXE_FILENAME_WE}.bin")
#add_executable ("${TARGET_EXE_FILENAME_WE}.bin" EXCLUDE_FROM_ALL "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_EXE_FILENAME_WE}.elf")
#set_target_properties ("${TARGET_EXE_FILENAME_WE}.bin" PROPERTIES LINKER_LANGUAGE ELF_TO_BIN)
#add_dependencies ("${TARGET_EXE_FILENAME_WE}.bin" "${TARGET_EXE_FILENAME_WE}.elf")


if (EXISTS "${BOARD_DIR}/toolchain_additional_targets.cmake")

include ("${BOARD_DIR}/toolchain_additional_targets.cmake")

endif ()

endmacro()

endif ()
