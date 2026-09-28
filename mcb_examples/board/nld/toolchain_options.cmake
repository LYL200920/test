
option (BOARD_NO_CRC "Disable usage of hardware CRC unit" OFF)
option (BOARD_USE_DEBUG_USART "Use USART2 for debugging" ON)
option (BOARD_ASSERT_MSG_NO_FUNCNAME "Omit function name from assert message to save space" ON)
option (BOARD_NO_RESET_HW_INIT "Omit hardware initialization at board reset" OFF)
option (BOARD_MCU_COMMS_ONLY "Use MCU and communication peripherals only" OFF)

set (BOARD_TYPE_ID "NLD" CACHE STRING "Board Type ID")
add_global_toolchain_definitions (-DBOARD_TYPE_ID_NLD)

if (BOARD_NO_CRC)
  add_global_toolchain_definitions (-DBOARD_NO_CRC)
endif ()

if (BOARD_USE_DEBUG_USART)
  add_global_toolchain_definitions (-DBOARD_USE_DEBUG_USART)
endif ()

if (BOARD_ASSERT_MSG_NO_FUNCNAME)
  add_global_toolchain_definitions (-DBOARD_ASSERT_MSG_NO_FUNCNAME)
endif ()

if (BOARD_NO_RESET_HW_INIT)
  add_global_toolchain_definitions (-DBOARD_NO_RESET_HW_INIT)
endif ()

if (BOARD_MCU_COMMS_ONLY)
  add_global_toolchain_definitions (-DBOARD_MCU_COMMS_ONLY)
endif ()


set (BOARD_VARIANT "A-03" CACHE STRING "Board Variant")
set_property (CACHE BOARD_VARIANT PROPERTY STRINGS A-02 A-03 A-04-STM32 A-04-GD32)

if (${BOARD_VARIANT} STREQUAL "A-02")
  add_global_toolchain_definitions (-DBOARD_VARIANT_A02)
  add_global_toolchain_definitions (-DBOARD_VARIANT_STM32)
elseif (${BOARD_VARIANT} STREQUAL "A-03")
  add_global_toolchain_definitions (-DBOARD_VARIANT_A03)
  add_global_toolchain_definitions (-DBOARD_VARIANT_STM32)
elseif (${BOARD_VARIANT} STREQUAL "A-04-STM32")
  add_global_toolchain_definitions (-DBOARD_VARIANT_A04)
  add_global_toolchain_definitions (-DBOARD_VARIANT_A04_STM32)
  add_global_toolchain_definitions (-DBOARD_VARIANT_STM32)
elseif (${BOARD_VARIANT} STREQUAL "A-04-GD32")
  add_global_toolchain_definitions (-DBOARD_VARIANT_A04)
  add_global_toolchain_definitions (-DBOARD_VARIANT_A04_GD32)
  add_global_toolchain_definitions (-DBOARD_VARIANT_GD32)
else ()
  message (FATAL_ERROR "unspecified board variant")
endif ()


# reserve 2x 1 KByte MCU flash pages as data flash

if (${BOARD_VARIANT} STREQUAL "A-04-GD32")
  set (TARGET_CPU		CORTEX-M3	CACHE STRING "")
  set (TARGET_ROM_SIZE		"1024*(32-2)"	CACHE STRING "")
else ()
  set (TARGET_CPU		CORTEX-M0	CACHE STRING "")
  set (TARGET_ROM_SIZE		"1024*(32-2)"	CACHE STRING "")
endif ()

set (TARGET_ENDIAN		little		CACHE STRING "")
set (TARGET_ROM_START		"0x00000000"	CACHE STRING "")
set (TARGET_RAM_START		"0x20000000"	CACHE STRING "")
set (TARGET_RAM_SIZE		"1024*8"	CACHE STRING "")
set (TARGET_ISTACK_SIZE "1024" CACHE STRING "istack size in bytes")
set (TARGET_USTACK_SIZE "1024" CACHE STRING "ustack size in bytes")

