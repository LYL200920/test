#!/bin/bash
#
# upload the specified ELF or MOT file using Jutze TFTP boot loader.

TOOLS_DIR="${1}"
IMG_FILE="${2}"

if [ -z ${TFTP_TARGET_IP+x} ]; then
  echo "TFTP_TARGET_IP not set.  Using default address 192.168.0.80.";
  TFTP_TARGET_IP="192.168.0.80"
else
  echo "TFTP_TARGET_IP = $TFTP_TARGET_IP";
fi

# reset the board into boot-mode
(echo "bootmode"; sleep 0.25 ) | telnet $TFTP_TARGET_IP 2> /dev/null

# wait a bit until ethernet comes up
sleep 1.75

# flash it
if type atftp &> /dev/null ; then
  time atftp --put --local-file ${IMG_FILE} --remote-file "user_flash.mot" --option "tsize `stat -c %s ${IMG_FILE}`" --option "blksize 1024" --tftp-timeout 1 $TFTP_TARGET_IP
else
  time (echo rexmt 1 ; echo put ${IMG_FILE} user_flash.mot ) | tftp -m binary $TFTP_TARGET_IP
fi

# board will sel-reset after flashing
