#message ("current list dir: ${CMAKE_CURRENT_LIST_DIR}")

include (${CMAKE_CURRENT_LIST_DIR}/make_romfs.cmake)

option (FS_FLASH_BFS_READ_ONLY "build fs::flash_bfs only with read-only support" OFF)
option (FS_FLASH_BFS_DISABLE_FILENAMES "build fs::flash_bfs without filename support" OFF)
option (FS_FLASH_BFS_DISABLE_BLOCK_READ "build fs::flash_bfs without the function file::read (fs::file_size_t start_block, size_t block_count, void* out)" OFF)

if (FS_FLASH_BFS_READ_ONLY)
  add_global_toolchain_definitions (-DFS_FLASH_BFS_READ_ONLY)
endif ()

if (FS_FLASH_BFS_DISABLE_FILENAMES)
  add_global_toolchain_definitions (-DFS_FLASH_BFS_DISABLE_FILENAMES)
endif ()

if (FS_FLASH_BFS_DISABLE_BLOCK_READ)
  add_global_toolchain_definitions (-DFS_FLASH_BFS_DISABLE_BLOCK_READ)
endif ()
