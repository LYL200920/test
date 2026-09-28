
option (BOARD_NO_CRC "Disable usage of hardware CRC unit" OFF)
option (BOARD_USE_DEBUG_USART "Use USART0 for debugging" ON)
option (BOARD_RCU_USE_IRC8M "Use IRC8M for board clock" OFF)
option (BOARD_ASSERT_MSG_NO_FUNCNAME "Omit function name from assert message to save space" ON)
option (BOARD_NO_RESET_HW_INIT "Omit hardware initialization at board reset" OFF)
option (BOARD_MCU_COMMS_ONLY "Use MCU and communication peripherals only" OFF)

if (BOARD_NO_CRC)
  add_global_toolchain_definitions (-DBOARD_NO_CRC)
endif ()

if (BOARD_USE_DEBUG_USART)
  add_global_toolchain_definitions (-DBOARD_USE_DEBUG_USART)
endif ()

if (BOARD_RCU_USE_IRC8M)
  add_global_toolchain_definitions (-DBOARD_RCU_USE_IRC8M)
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

#-----------------------------------------------------------------------------------------------
# 256K flash:
#  - main program                : 64K
#  - firmware updater            : 64K
#  - firmware updater work flash : 64K == firmware updater
#  - reserve                     : 58K
#  - data                        :  6K

# reserve 3x2 KByte MCU flash pages as data flash
set (BOARD_VARIANT 		GD32F303RCT6A		CACHE STRING "")
set (TARGET_CPU			CORTEX-M4-HARDFP	CACHE STRING "")
set (TARGET_ENDIAN		little			CACHE STRING "")
set (TARGET_ROM_SIZE		"1024*(256-6)"		CACHE STRING "")
set (TARGET_ROM_START		"0x00000000"		CACHE STRING "")
set (TARGET_RAM_START		"0x20000000"		CACHE STRING "")
set (TARGET_RAM_SIZE		"1024*48"		CACHE STRING "")
set (TARGET_ISTACK_SIZE "1024*8" CACHE STRING "istack size in bytes")
set (TARGET_USTACK_SIZE "1024*8" CACHE STRING "ustack size in bytes")