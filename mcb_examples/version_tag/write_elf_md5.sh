#!/bin/sh

ELF_FILE=${1}
TARGET_NM=${2}
TARGET_OBJDUMP=${3}

MD5_SUM=`md5sum -b "${ELF_FILE}" | awk '{print $1}'`

MD5_START_ADDR=`${TARGET_NM} "${ELF_FILE}" | grep __version_tag_build_md5 | awk '{print $1}'`
MD5_START_ADDR="0x${MD5_START_ADDR}"
MD5_END_ADDR=`printf '0x%x' $((${MD5_START_ADDR} + 1))`

# the following objdump line
#   ${TARGET_OBJDUMP} -d --file-offsets --start-address=${MD5_START_ADDR} --stop-address=${MD5_END_ADDR} "${ELF_FILE}"
#
# will print something like this
#
# li5000_mcb.elf:     file format elf32-rx-le
#
#
# Disassembly of section .text:
#
# fffa00c4 <__version_tag_build_md5> (File Offset: 0x10c4):
# fffa00c4:	30                            	*unknown*
#
# we're interested in the file offset number only

MD5_FILE_OFFSET=`${TARGET_OBJDUMP} -d --file-offsets \
  --start-address=${MD5_START_ADDR} --stop-address=${MD5_END_ADDR} "${ELF_FILE}" \
  | grep -o "File Offset: 0x.[0-9a-f]*" | awk '{print $3}'`

echo "MD5 sum: ${MD5_SUM}"
echo "MD5 image address: ${MD5_START_ADDR}"
echo "MD5 file offset: ${MD5_FILE_OFFSET}"

echo -n "${MD5_SUM}" | dd of="${ELF_FILE}" bs=1 seek=$((${MD5_FILE_OFFSET})) conv=notrunc status=none
