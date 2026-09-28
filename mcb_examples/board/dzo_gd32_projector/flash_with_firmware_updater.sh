#!/bin/bash
#
# upload the specified BIN file using STM32 USART boot mode and a USB UART

IMG_FILE="${1}"
THIS_DIR="${PWD}"

if [ -z ${TARGET_RS232_FLASH_DEV+x} ]; then
  echo "TARGET_RS232_FLASH_DEV not set.  Using default /dev/ttyUSB0.";
  TARGET_RS232_FLASH_DEV="/dev/ttyUSB0"
else
  echo "TARGET_RS232_FLASH_DEV = $TARGET_RS232_FLASH_DEV";
fi

# make sure that ctrl-c will exit this script and not get stuck in infinite retry loop.
stty -echoctl # hide ^C
trap 'exit 1' SIGINT

# Baud rate
# 57600
# 115200
# 128000
# 230400
# 256000
# 460800 // for GD32 this one the best.
# 500000
# 576000
# 921600
# 1000000
# 1152000
# 1500000

# use the same speed as board's default debug uart speed setting.
# this allows using stm32link for flashing while having the debug uart terminal
# open at the same time.
BAUD_RATE=500000

# reset, switch to bootloader-mode, flash, switch to normal more, reset.
# erase only first 64 2-KB flash pages, keep the last 3 2-KB pages which store
# the configuration.

# 256K flash:
#  - main program                : 32  2-KBytes
#  - firmware updater            : 32  2-KBytes
#  - firmware updater work flash : 32  2-KBytes == firmware updater
#  - reserve                     : 29  2-KBytes
#  - data                        :  3  2-KBytes
WRITE_SIZE=64

while true; do
  stm32flash -R -b ${BAUD_RATE} \
		-w "${IMG_FILE}" \
		-e ${WRITE_SIZE} \
		-i "rts&-dtr,,-rts,,,:rts&dtr,-rts"  \
		$TARGET_RS232_FLASH_DEV

  if [ $? -eq 0 ]; then
    break
  fi

  # try alternative with flipped DTR polarity
  echo "trying alternate DTR polarity.."
  stm32flash -R -b ${BAUD_RATE} \
		-w "${IMG_FILE}" \
		-e ${WRITE_SIZE} \
		-i "rts&dtr,,-rts,,,:rts&-dtr,-rts"  \
		$TARGET_RS232_FLASH_DEV

  if [ $? -eq 0 ]; then
    break
  fi

  echo "retrying ..."
done
