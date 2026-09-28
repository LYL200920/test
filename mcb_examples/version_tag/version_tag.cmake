
# invalidate the generated product version source file to force the re-generation
# on each build.  otherwise we might miss changes to the repository.  e.g.
# a new commit or tag is added after the build.  the next rebuild will come out
# clean and will not pick up the changes in the repo.
#
# however, doing so will force re-linking every time.  this can be annoying
# when using LTO (long link time) and re-uploading the image via make *.upload.
#

#macro (add_product_version _target _product_name _copyright)

#add_custom_command (
#  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_target}.product_version.S
#  COMMAND ${CMAKE_SOURCE_DIR}/../gitrev.sh -p \"${_product_name}\" -c \"${_copyright}\" -t asm > ${CMAKE_CURRENT_BINARY_DIR}/${_target}.product_version.S
#  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#)

#add_library (${_target}.product_version STATIC
#  ${CMAKE_CURRENT_BINARY_DIR}/${_target}.product_version.S
#)

## the product_version section is not referenced by anything.  thus by default
## it will not be pulled in by the linker from the library into the final
## ELF.  to force the inclusion use "whole-archive".
#target_link_libraries (${_target}
# -Wl,-whole-archive ${_target}.product_version -Wl,-no-whole-archive
#)

#add_custom_command (TARGET ${_target}.product_version
#  POST_BUILD COMMAND -mv ${CMAKE_CURRENT_BINARY_DIR}/${_target}.product_version.S ${CMAKE_CURRENT_BINARY_DIR}/${_target}.product_version.S.prev
#)

#endmacro ()


# to avoid re-linking if the version tag hasn't changed, split the generation
# of the version tag source files into two steps.
# in the first step, get the product tag source into a .new file.
# in the second step, if the .new file is different from the current file
# copy it.  this will not rebuild the version tag library if the contents
# have not changed and thus the application image will also not be rebuilt.

enable_language(ASM)

set (VERSION_TAG_REV_TOOL ${CMAKE_CURRENT_LIST_DIR}/gitrev.sh)
set (WRITE_ELF_MD5_TOOL ${CMAKE_CURRENT_LIST_DIR}/write_elf_md5.sh)

macro (set_default_version_tag_product_name _val)
set (TARGET_VERSION_PRODUCT_NAME "${_val}" CACHE STRING "version tag product name")
endmacro ()

macro (set_default_version_tag_copyright_str _val)
set (TARGET_VERSION_COPYRIGHT "${_val}" CACHE STRING "version tag copyright string")
endmacro ()

macro (add_version_tag _target)

get_filename_component (TARGET_EXE_FILENAME_WE ${_target} NAME_WE)
get_filename_component (TARGET_EXE_FILENAME ${_target} NAME)

add_custom_command (
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_target}.version_tag.S.new
#  PRE_LINK

  COMMAND ${VERSION_TAG_REV_TOOL} -p \"${TARGET_VERSION_PRODUCT_NAME}\" -c \"${TARGET_VERSION_COPYRIGHT}\" -t asm > ${CMAKE_CURRENT_BINARY_DIR}/${_target}.version_tag.S.new

  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_custom_command (
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_target}.version_tag.S
#  PRE_LINK
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/${_target}.version_tag.S.new ${CMAKE_CURRENT_BINARY_DIR}/${_target}.version_tag.S

  # always remove the .new file to force its re-generation.
  COMMAND ${CMAKE_COMMAND} -E remove -f ${CMAKE_CURRENT_BINARY_DIR}/${_target}.version_tag.S.new
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${_target}.version_tag.S.new
)

add_library (${_target}.version_tag STATIC
  ${CMAKE_CURRENT_BINARY_DIR}/${_target}.version_tag.S
)

# the product_version section is not referenced by anything.  thus by default
# it will not be pulled in by the linker from the library into the final
# ELF.  to force the inclusion use "whole-archive".
target_link_libraries (${_target}
 -Wl,-whole-archive ${_target}.version_tag -Wl,-no-whole-archive
)

# after building the ELF calculate the MD5 checksum and overwrite it in the
# ELF file.
# the version tag contains a symbol __version_tag_build_md5 which indicates the
# start of the MD5 string in the version tag data.  we can find the address
# and the file offset of that symbol in the ELF file and overwrite it with
# the new contents.
add_custom_command (TARGET ${_target}
  POST_BUILD
  COMMAND ${WRITE_ELF_MD5_TOOL} "${TARGET_EXE_FILENAME}" ${CMAKE_NM} ${CMAKE_OBJDUMP}
  COMMENT "Writing ELF MD5 checksum to version tag"
)

endmacro ()




#
# for each listed library in the project, extract the version section,
# make a big version file out of it
#
#
# after building a library, generate a version tag and add it to the library
# archive.
#
# before building the application, extract version tags from each library
# and merge them in one section.  then prefix with the application version
# tag and add it to the application for final linking
#
# extract one section (objcopy will output error messages for each archived
# object if it doesn't have the section)
#   rx-elf-objcopy --dump-section .text.product_version=-  mcb_lib/libmcb_lib.a 3> /dev/null
#

