
option (LOGGING_LOGGING_ENABLE "enable logging library" ON)

if (LOGGING_LOGGING_ENABLE)
  add_global_toolchain_definitions (-DLOGGING_LOGGING_ENABLE)
endif ()
