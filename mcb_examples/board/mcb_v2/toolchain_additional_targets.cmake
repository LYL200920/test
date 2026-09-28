
add_custom_target ("${TARGET_EXE_FILENAME_WE}.upload_tftp"
  COMMAND ${BOARD_DIR}/../mcb_v1/tools/upload_tftp.sh "${BOARD_DIR}/../mcb_v1/tools/" "${TARGET_EXE_FILENAME_WE}.mot"
  DEPENDS ${TARGET_EXE_FILENAME_WE}.mot
)


add_custom_target ("${TARGET_EXE_FILENAME_WE}.upload_tftp2"
  COMMAND ${BOARD_DIR}/../mcb_v1/tools/upload_tftp2.sh "${BOARD_DIR}/../mcb_v1/tools/" "${TARGET_EXE_FILENAME_WE}.mot"
  DEPENDS ${TARGET_EXE_FILENAME_WE}.mot
)

