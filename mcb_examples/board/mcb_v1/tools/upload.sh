#!/bin/bash
#
# upload the specified ELF or MOT file using RX SCI1 boot mode and a serial
# cable connected as USB1

TOOLS_DIR="${1}"
IMG_FILE="${2}"
THIS_DIR="${PWD}"

if [ ! -f "${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" ]; then
  cd "${TOOLS_DIR}/serial_io_ctrl"
  make
fi

if [ ! -f "${TOOLS_DIR}/dj-flash-tool/rxs" ]; then
  cd "${TOOLS_DIR}/dj-flash-tool"
  make rxs
fi

cd "${THIS_DIR}"

if [ -z ${TARGET_RS232_DEBUG_DEV+x} ]; then
  echo "TARGET_RS232_DEBUG_DEV not set.  Using default /dev/ttyUSB0.";
  TARGET_RS232_DEBUG_DEV="/dev/ttyUSB0"
else
  echo "TARGET_RS232_DEBUG_DEV = $TARGET_RS232_DEBUG_DEV";
fi

if [ -z ${TARGET_RS232_FLASH_DEV+x} ]; then
  echo "TARGET_RS232_FLASH_DEV not set.  Using default /dev/ttyUSB1.";
  TARGET_RS232_FLASH_DEV="/dev/ttyUSB1"
else
  echo "TARGET_RS232_FLASH_DEV = $TARGET_RS232_FLASH_DEV";
fi


# reset the board into boot-mode
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 1 1
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 1 0

# wait a bit until it comes up
sleep 0.25

# flash it
"${TOOLS_DIR}/dj-flash-tool/rxs" -v -b 115200 -p $TARGET_RS232_FLASH_DEV "${IMG_FILE}"

# release reset
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 1 1
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 0 1
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 0 0
