#!/bin/bash
#
# upload the specified ELF or MOT file using Jutze TFTP boot loader.

TOOLS_DIR="${1}"
IMG_FILE="${2}"
THIS_DIR="${PWD}"

if [ ! -f "${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" ]; then
  cd "${TOOLS_DIR}/serial_io_ctrl"
  make
fi

cd "${THIS_DIR}"

if [ -z ${TFTP_TARGET_IP+x} ]; then
  echo "TFTP_TARGET_IP not set.  Using default address 192.168.0.80.";
  TFTP_TARGET_IP="192.168.0.80"
else
  echo "TFTP_TARGET_IP = $TFTP_TARGET_IP";
fi

if [ -z ${TARGET_RS232_DEBUG_DEV+x} ]; then
  echo "TARGET_RS232_DEBUG_DEV not set.  Using default /dev/ttyUSB0.";
  TARGET_RS232_DEBUG_DEV="/dev/ttyUSB0"
else
  echo "TARGET_RS232_DEBUG_DEV = $TARGET_RS232_DEBUG_DEV";
fi

# reset the board into boot-mode
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 1 1
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 1 0

# wait a bit until ethernet comes up
sleep 1.75

# flash it
if type atftp &> /dev/null ; then
  time atftp --put --local-file ${IMG_FILE} --remote-file "user_flash.mot" --option "tsize `stat -c %s ${IMG_FILE}`" --option "blksize 1024" --tftp-timeout 1 $TFTP_TARGET_IP
else
  time (echo rexmt 1 ; echo put ${IMG_FILE} user_flash.mot ) | tftp -m binary $TFTP_TARGET_IP
fi

# release reset
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 1 1
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 0 1
"${TOOLS_DIR}/serial_io_ctrl/serial_io_ctrl" $TARGET_RS232_DEBUG_DEV 0 0
