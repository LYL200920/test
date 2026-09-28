
add_custom_target ("${TARGET_EXE_FILENAME_WE}.flash"
  COMMAND ${BOARD_DIR}/flash.sh "${TARGET_EXE_FILENAME_WE}.bin"
  DEPENDS ${TARGET_EXE_FILENAME_WE}.bin
)

