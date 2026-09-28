
set (TARGET_ROM_END "0xFFFFFFFF" CACHE STRING "ROM end address")
set (TARGET_RAM_START "0x00000100" CACHE STRING "RAM start address")
set (TARGET_ISTACK_SIZE "1024" CACHE STRING "istack size in bytes")
set (TARGET_USTACK_SIZE "1024*2" CACHE STRING "ustack size in bytes")

option (TARGET_RX_USER_BOOT_IMAGE "build a user boot image" OFF)
option (TARGET_RX_TEXT_IN_RAM "the .text section is located in RAM instead of ROM" OFF)

set (BOARD_VARIANT "RX63N_144" CACHE STRING "Board Variant")
set_property (CACHE BOARD_VARIANT PROPERTY STRINGS RX631_144 RX63N_144 RX64M_144 RX71M_144 RX64M_176 RX71M_176)

if (TARGET_RX_USER_BOOT_IMAGE)
  add_global_toolchain_definitions (-DRX_USER_BOOT_IMAGE)
endif ()

if (TARGET_RX_TEXT_IN_RAM)
  add_global_toolchain_definitions (-DRX_TEXT_IN_RAM)
endif ()

if (${BOARD_VARIANT} STREQUAL "RX631_144")
  add_global_toolchain_definitions (-DMCB_USE_RX631_144)
  set (TARGET_ROM_SIZE "1024*256" CACHE STRING "ROM size in bytes")
  set (TARGET_RAM_SIZE "1024*128-256" CACHE STRING "RAM size in bytes")

elseif (${BOARD_VARIANT} STREQUAL "RX63N_144")
  add_global_toolchain_definitions (-DMCB_USE_RX63N_144)
  set (TARGET_ROM_SIZE "1024*768" CACHE STRING "ROM size in bytes")
  set (TARGET_RAM_SIZE "1024*128-256" CACHE STRING "RAM size in bytes")

elseif (${BOARD_VARIANT} STREQUAL "RX64M_144")
  add_global_toolchain_definitions (-DMCB_USE_RX64M_144)
  set (TARGET_ROM_SIZE "1024*1024*2" CACHE STRING "ROM size in bytes")
  set (TARGET_RAM_SIZE "1024*512-256" CACHE STRING "RAM size in bytes")

elseif (${BOARD_VARIANT} STREQUAL "RX71M_144")
  add_global_toolchain_definitions (-DMCB_USE_RX71M_144)
  set (TARGET_ROM_SIZE "1024*1024*2" CACHE STRING "ROM size in bytes")
  set (TARGET_RAM_SIZE "1024*512-256" CACHE STRING "RAM size in bytes")

elseif (${BOARD_VARIANT} STREQUAL "RX64M_176")
  add_global_toolchain_definitions (-DMCB_USE_RX64M_176)
  set (TARGET_ROM_SIZE "1024*1024*2" CACHE STRING "ROM size in bytes")
  set (TARGET_RAM_SIZE "1024*512-256" CACHE STRING "RAM size in bytes")

elseif (${BOARD_VARIANT} STREQUAL "RX71M_176")
  add_global_toolchain_definitions (-DMCB_USE_RX71M_176)
  set (TARGET_ROM_SIZE "1024*1024*2" CACHE STRING "ROM size in bytes")
  set (TARGET_RAM_SIZE "1024*512-256" CACHE STRING "RAM size in bytes")

else ()
  message (FATAL_ERROR "undefined board variant")

endif ()

option (BOARD_USE_INTERNAL_I2C "Enable on-board internal I2C on SCI2" ON)
option (BOARD_USE_INTERNAL_SPI "Enable on-board internal SPI on SCI4" OFF)
option (BOARD_USE_DAC124S085 "Enable the DAC124S085 device" OFF)
option (BOARD_USE_RS485 "Enable RS485 ports" ON)
option (BOARD_USE_PCA9698 "Enable the PCA9698 devices" ON)
option (BOARD_USE_PCD4641 "Enable the PCD4641 device" ON)
option (BOARD_USE_MCX514 "Enable the MCX514 device" ON)
option (BOARD_USE_USB0 "Enable the USB0 fucntion device" OFF)
option (BOARD_USE_USBA "Enable the USBA host device" OFF)
option (BOARD_USE_TRIGGER_IO "Enable the trigger IO device" ON)
option (BOARD_USE_LEDS "Enable the LED outputs device" ON)
option (BOARD_USE_DIPSWITCH "Enable the dipswitch inputs device" ON)
option (BOARD_USE_DIGITAL_IO "Enable the digital IO device" ON)
option (BOARD_USE_ETHERC0 "Enable the RX EtherC device as eth0" ON)
option (BOARD_USE_ETHERC0_RAW "Enable the RX EtherC device as raw device" OFF)
option (BOARD_USE_ETHERC1 "Enable the RX EtherC device as eth1" ON)
option (BOARD_USE_ETHERC1_RAW "Enable the RX EtherC device as raw device" OFF)
option (BOARD_USE_LAN9250 "Enable the LAN9250 device as eth2" ON)
option (BOARD_USE_LAN9250_RAW "Enable the LAN9250 device as raw device" OFF)
option (BOARD_USE_FCU "Enable the FCU device" ON)
option (BOARD_USE_SCI_DEBUG "Use SCI1 as debug printf output" ON)
option (BOARD_USE_SCI0_USART "Enable general purpose serial port on SCI0" ON)
option (BOARD_USE_SCI3_USART "Enable general purpose serial port on SCI3" ON)
option (BOARD_USE_SCI5_USART "Enable general purpose serial port on SCI5" ON)
option (BOARD_USE_TPU "Enable the TPU device" ON)
option (BOARD_NO_RX_CRC "Do not use RX hardware CRC calculator" OFF)
option (BOARD_NO_RESET_HW_INIT "Do not initialize hardware at reset" OFF)
option (BOARD_NO_PERIPHERAL_RESET_RELEASE "Do not assert or release the peripheral reset line during and after board initialization" OFF)

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
  add_global_toolchain_definitions (-DMCB_USE_USB_FUNCTION)
endif ()

if (BOARD_USE_USBA)
  add_global_toolchain_definitions (-DMCB_USE_USB_HOST)
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

if (BOARD_USE_ETHERC0)
  add_global_toolchain_definitions (-DMCB_USE_RX_ETHERC0)
endif ()

if (BOARD_USE_ETHERC0_RAW)
  add_global_toolchain_definitions (-DMCB_USE_RX_ETHERC0_RAW)
endif ()

if (BOARD_USE_ETHERC1)
  add_global_toolchain_definitions (-DMCB_USE_RX_ETHERC1)
endif ()

if (BOARD_USE_ETHERC1_RAW)
  add_global_toolchain_definitions (-DMCB_USE_RX_ETHERC1_RAW)
endif ()

if (BOARD_USE_LAN9250)
  add_global_toolchain_definitions (-DMCB_USE_LAN9250)
endif ()

if (BOARD_USE_LAN9250_RAW)
  add_global_toolchain_definitions (-DMCB_USE_LAN9250_RAW)
endif ()

if (BOARD_USE_FCU)
  add_global_toolchain_definitions (-DMCB_USE_FCU)
endif ()

if (BOARD_USE_SCI_DEBUG)
  add_global_toolchain_definitions (-DMCB_USE_SCI_DEBUG)
endif ()

if (BOARD_USE_SCI0_USART)
  add_global_toolchain_definitions (-DMCB_USE_SCI0_USART)
endif ()

if (BOARD_USE_SCI3_USART)
  add_global_toolchain_definitions (-DMCB_USE_SCI3_USART)
endif ()

if (BOARD_USE_SCI5_USART)
  add_global_toolchain_definitions (-DMCB_USE_SCI5_USART)
endif ()

if (BOARD_USE_TPU)
  add_global_toolchain_definitions (-DMCB_USE_TPU)
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

