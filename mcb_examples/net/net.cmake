
option (NET_NO_TCP "build networking libraries without TCP support" OFF)
option (NET_SLLP_DISABLE_FRAME_11 "disable support for frame format 11" OFF)

if (NET_NO_TCP)
  add_global_toolchain_definitions (-DNET_NO_TCP)
endif ()

if (NET_SLLP_DISABLE_FRAME_11)
  add_global_toolchain_definitions (-DNET_SLLP_DISABLE_FRAME_11)
endif ()
