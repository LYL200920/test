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

# reset, switch to bootloader-mode, flash, switch to normal more, reset
# erase only first 30 1-KB flash pages, keep the last 2 1-KB pages which store
# the configuration.
while true; do
  stm32flash -R -b 115200 -w "${IMG_FILE}" -e 30 -i "rts,-dtr,-rts,:rts,dtr,-rts"  $TARGET_RS232_FLASH_DEV

  if [ $? -eq 0 ]; then
    break
  fi

  # try alternative with flipped DTR polarity
  echo "trying alternate DTR polarity.."
  stm32flash -R -b 115200 -w "${IMG_FILE}" -e 30 -i "rts,dtr,-rts,dtr:rts,-dtr,-rts"  $TARGET_RS232_FLASH_DEV

  if [ $? -eq 0 ]; then
    break
  fi

  echo "retrying ..."
done
