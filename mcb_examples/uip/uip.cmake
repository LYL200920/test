
option (NET_NO_TCP "build networking libraries without TCP support" OFF)

if (NET_NO_TCP)
  add_global_toolchain_definitions (-DNET_NO_TCP)
endif ()
