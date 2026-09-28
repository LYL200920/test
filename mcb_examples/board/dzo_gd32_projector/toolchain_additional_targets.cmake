
add_custom_target ("${TARGET_EXE_FILENAME_WE}.flash"
  COMMAND ${BOARD_DIR}/flash.sh "${TARGET_EXE_FILENAME_WE}_base.bin"
  DEPENDS ${TARGET_EXE_FILENAME_WE}_base.bin
)

add_custom_target ("${TARGET_EXE_FILENAME_WE}.flash_with_firmware_updater"
  COMMAND ${BOARD_DIR}/flash_with_firmware_updater.sh "${TARGET_EXE_FILENAME_WE}.bin"
  DEPENDS ${TARGET_EXE_FILENAME_WE}.bin
)

add_custom_target ("${TARGET_EXE_FILENAME_WE}.read_flash"
  COMMAND ${BOARD_DIR}/read_flash.sh "read_from_chip.bin"
)

add_custom_target ("${TARGET_EXE_FILENAME_WE}.erase_flash"
  COMMAND ${BOARD_DIR}/erase_flash.sh
)

