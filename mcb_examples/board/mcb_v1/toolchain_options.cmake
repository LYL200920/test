set (TARGET_CPU "RX600" CACHE STRING "CPU Type")

if (TARGET_RX_USER_BOOT_IMAGE)
  add_global_toolchain_definitions (-DRX_USER_BOOT_IMAGE)
  set (TARGET_ROM_SIZE "1024*16" CACHE STRING "ROM size in bytes")
  set (TARGET_ROM_END "0xFF7FFFFF" CACHE STRING "ROM end address")
else ()
  set (TARGET_ROM_SIZE "1024*768" CACHE STRING "ROM size in bytes")
  set (TARGET_ROM_END "0xFFFFFFFF" CACHE STRING "ROM end address")
endif ()

set (TARGET_RAM_START "0x00000100" CACHE STRING "RAM start address")
set (TARGET_RAM_SIZE "1024*128-256" CACHE STRING "RAM size in bytes")
set (TARGET_ISTACK_SIZE "1024" CACHE STRING "istack size in bytes")
set (TARGET_USTACK_SIZE "1024*2" CACHE STRING "ustack size in bytes")

option (TARGET_RX_USER_BOOT_IMAGE "build a user boot image" OFF)
option (TARGET_RX_TEXT_IN_RAM "the .text section is located in RAM instead of ROM" OFF)

set (BOARD_TYPE_ID "MCB1" CACHE STRING "Board Type ID")
add_global_toolchain_definitions (-DBOARD_TYPE_ID_MCB1)

if (TARGET_RX_TEXT_IN_RAM)
  add_global_toolchain_definitions (-DRX_TEXT_IN_RAM)
endif ()

option (BOARD_USE_INTERNAL_I2C "Enable on-board internal I2C on SCI2" ON)
option (BOARD_USE_INTERNAL_SPI "Enable on-board internal SPI on SCI4" OFF)
option (BOARD_USE_DAC124S085 "Enable the DAC124S085 device" OFF)
option (BOARD_USE_RS485 "Enable RS485 ports" ON)
option (BOARD_USE_PCA9698 "Enable the PCA9698 devices" ON)
option (BOARD_USE_PCD4641 "Enable the PCD4641 device" ON)
option (BOARD_USE_MCX514 "Enable the MCX514 device" ON)
option (BOARD_USE_USB0 "Enable the USB0 device" OFF)
option (BOARD_USE_TRIGGER_IO "Enable the trigger IO device" ON)
option (BOARD_USE_LEDS "Enable the LED outputs device" ON)
option (BOARD_USE_DIPSWITCH "Enable the dipswitch inputs device" ON)
option (BOARD_USE_DIGITAL_IO "Enable the digital IO device" ON)
option (BOARD_USE_ETHERC "Enable the RX EtherC device as eth0" ON)
option (BOARD_USE_ETHERC_RAW "Enable the RX EtherC device as raw device" OFF)
option (BOARD_USE_FCU "Enable the FCU device" ON)
option (BOARD_USE_SCI_DEBUG "Use SCI1 as debug printf output" ON)
option (BOARD_USE_SCI0_RS232C "Enable RS232 on SCI0" ON)
option (BOARD_USE_TPU "Enable the TPU device" ON)
option (BOARD_USE_DTC "Enable the DTC device" ON)
option (BOARD_NO_RX_CRC "Do not use RX hardware CRC calculator" OFF)
option (BOARD_NO_RESET_HW_INIT "Do not initialize hardware at reset" OFF)
option (BOARD_NO_PERIPHERAL_RESET_RELEASE "Do not assert or release the peripheral reset line during and after board initialization" OFF)
option (BOARD_ASSERT_MSG_NO_FUNCNAME "Omit function name from assert message to save space" ON)

add_global_toolchain_definitions (-DMCB_USE_RX63)
add_global_toolchain_definitions (-DMCB_USE_RX631_144)

if (BOARD_USE_INTERNAL_I2C)
  add_global_toolchain_definitions (-DMCB_USE_I2C)
endif ()

if (BOARD_USE_INTERNAL_SPI)
  add_global_toolchain_definitions (-DMCB_USE_SPI)
endif ()

if (BOARD_USE_DAC124S085)
  add_global_toolchain_definitions (-DMCB_USE_DAC124S085)
endif ()

if (BOARD_USE_RS485)
  add_global_toolchain_definitions (-DMCB_USE_RS485)
endif ()

if (BOARD_USE_PCA9698)
  add_global_toolchain_definitions (-DMCB_USE_PCA9698)
endif ()

if (BOARD_USE_PCD4641)
  add_global_toolchain_definitions (-DMCB_USE_PCD4641)
endif ()

if (BOARD_USE_MCX514)
  add_global_toolchain_definitions (-DMCB_USE_MCX514)
endif ()

if (BOARD_USE_USB0)
  add_global_toolchain_definitions (-DMCB_USE_USB)
endif ()

if (BOARD_USE_TRIGGER_IO)
  add_global_toolchain_definitions (-DMCB_USE_TRIGGER_IO)
endif ()

if (BOARD_USE_LEDS)
  add_global_toolchain_definitions (-DMCB_USE_LEDS)
endif ()

if (BOARD_USE_DIPSWITCH)
  add_global_toolchain_definitions (-DMCB_USE_DIPSWITCH)
endif ()

if (BOARD_USE_DIGITAL_IO)
  add_global_toolchain_definitions (-DMCB_USE_DIGITAL_IO)
endif ()

if (BOARD_USE_ETHERC)
  add_global_toolchain_definitions (-DMCB_USE_ETHERC)
endif ()

if (BOARD_USE_ETHERC_RAW)
  add_global_toolchain_definitions (-DMCB_USE_ETHERC_RAW)
endif ()

if (BOARD_USE_FCU)
  add_global_toolchain_definitions (-DMCB_USE_FCU)
endif ()

if (BOARD_USE_SCI_DEBUG)
  add_global_toolchain_definitions (-DMCB_USE_SCI_DEBUG)
endif ()

if (BOARD_USE_SCI0_RS232C)
  add_global_toolchain_definitions (-DMCB_USE_SCI0_RS232C)
endif ()

if (BOARD_USE_TPU)
  add_global_toolchain_definitions (-DMCB_USE_TPU)
endif ()

if (BOARD_USE_DTC)
  add_global_toolchain_definitions (-DMCB_USE_DTC)
endif ()

if (BOARD_NO_RX_CRC)
  add_global_toolchain_definitions (-DMCB_NO_CRC)
endif ()

if (BOARD_NO_RESET_HW_INIT)
  add_global_toolchain_definitions (-DMCB_NO_RESET_HW_INIT)
endif ()

if (BOARD_NO_PERIPHERAL_RESET_RELEASE)
  add_global_toolchain_definitions (-DMCB_NO_PERIPHERAL_RESET_RELEASE)
endif ()

if (BOARD_ASSERT_MSG_NO_FUNCNAME)
  add_global_toolchain_definitions (-DBOARD_ASSERT_MSG_NO_FUNCNAME)
endif ()
