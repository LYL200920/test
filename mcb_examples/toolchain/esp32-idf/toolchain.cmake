
set (CMAKE_SYSTEM_NAME Generic)
set (CMAKE_SYSTEM_VERSION 1)
set (CMAKE_SYSTEM_PROCESSOR xtensa)
set (CMAKE_CROSSCOMPILING TRUE)

set (CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set (CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set (CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set (IDF_PATH $ENV{IDF_PATH})

if (NOT IDF_PATH OR IDF_PATH STREQUAL "")
  message (FATAL_ERROR "IDF_PATH environment variable not set")
endif ()

include ($ENV{IDF_PATH}/tools/cmake/toolchain-esp32.cmake)

set (TARGET_ENDIAN "little" CACHE STRING "Endian")

# ----------------------------------------------------------------------------

if (NOT CMAKE_TOOLCHAIN_FILE OR CMAKE_TOOLCHAIN_FILE STREQUAL "")

# don't add anything funny if the toolchain file is not set.
# this happens when cmake runs the initial "is compiler compiler working"
# checks.

else ()

function (rebuild_build_flags)
endfunction ()

include (${CMAKE_CURRENT_LIST_DIR}/../common.cmake)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

endif ()

