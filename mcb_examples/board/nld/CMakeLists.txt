
project (board)
enable_language(ASM)

if (NOT CMAKE_CXX_STANDARD)
  message (FATAL_ERROR "this library requires at least c++14")
endif ()

if (${CMAKE_CXX_STANDARD} LESS 14)
  message (FATAL_ERROR "this library requires at least c++14")
endif ()

board_toolchain_add_linkerscript ("linker_script.c")

# for details, see board_info_data.cpp
set_source_files_properties (board_info_data.cpp
  PROPERTIES COMPILE_FLAGS -fno-lto)

# FIXME: for some reason, the fvectors section in board.cpp is dropped
#        and the only way to get it in is to use whole-archive linking
#        of the board library in the main project.

add_target_library (board STATIC
  ../board_info.cpp
  board_info_data.cpp
  board.cpp
  board.S

  # the dummy linker script file to create a dependency on the actual
  # linker script.
  linker_script.c
)

# the C runtime support library is separated because some applications
# might not want to use it but something else, e.g. tiny apps like the
# tftp bootloader.
add_target_library (board_crt STATIC
  ../crt.cpp
)
