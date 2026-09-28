
# DJ's rxusb tool has problems when reading/uploading small ELFs (< 3KByte)
# but s-rec seems to be working fine.  Thus, always use s-rec.
add_custom_target ("${TARGET_EXE_FILENAME_WE}.upload"
  COMMAND ${BOARD_DIR}/tools/upload.sh "${BOARD_DIR}/tools" "${TARGET_EXE_FILENAME_WE}.mot"
  DEPENDS ${TARGET_EXE_FILENAME_WE}.mot
)

add_custom_target ("${TARGET_EXE_FILENAME_WE}.upload_usb"
  COMMAND ${BOARD_DIR}/tools/upload_usb.sh "${BOARD_DIR}/tools" "${TARGET_EXE_FILENAME_WE}.mot"
  DEPENDS ${TARGET_EXE_FILENAME_WE}.mot
)

add_custom_target ("${TARGET_EXE_FILENAME_WE}.upload_tftp"
  COMMAND ${BOARD_DIR}/tools/upload_tftp.sh "${BOARD_DIR}/tools" "${TARGET_EXE_FILENAME_WE}.mot"
  DEPENDS ${TARGET_EXE_FILENAME_WE}.mot
)

add_custom_target ("${TARGET_EXE_FILENAME_WE}.upload_tftp2"
  COMMAND ${BOARD_DIR}/tools/upload_tftp2.sh "${BOARD_DIR}/tools" "${TARGET_EXE_FILENAME_WE}.mot"
  DEPENDS ${TARGET_EXE_FILENAME_WE}.mot
)

add_custom_target ("${TARGET_EXE_FILENAME_WE}.upload_user_boot"
  COMMAND ${BOARD_DIR}/tools/upload_user_boot.sh "${BOARD_DIR}/tools" "${TARGET_EXE_FILENAME_WE}.mot"
  DEPENDS ${TARGET_EXE_FILENAME_WE}.mot
)
