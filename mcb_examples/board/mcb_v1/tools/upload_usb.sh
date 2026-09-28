#!/bin/bash
#
# upload the specified ELF or MOT file using RX USB boot mode
# (default Renesas USB boot loader)

TOOLS_DIR="${1}"
IMG_FILE="${2}"
THIS_DIR="${PWD}"

if [ ! -f "${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" ]; then
  cd "${TOOLS_DIR}/serial_io_ctrl"
  make
fi

if [ ! -f "${TOOLS_DIR}/dj-flash-tool/rxusb" ]; then
  cd "${TOOLS_DIR}/dj-flash-tool"
  make rxusb
fi

cd "${THIS_DIR}"

if [ -z ${TARGET_RS232_DEBUG_DEV+x} ]; then
  echo "TARGET_RS232_DEBUG_DEV not set.  Using default /dev/ttyUSB0.";
  TARGET_RS232_DEBUG_DEV="/dev/ttyUSB0"
else
  echo "TARGET_RS232_DEBUG_DEV = $TARGET_RS232_DEBUG_DEV";
fi

# reset the board into boot-mode
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 1 1
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 1 0

# wait a bit until USB comes up
sleep 0.5

# flash it
"${TOOLS_DIR}/dj-flash-tool/rxusb" -v "${IMG_FILE}"

# release reset
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 0 0
