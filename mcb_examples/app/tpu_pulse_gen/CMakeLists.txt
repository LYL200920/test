cmake_minimum_required(VERSION 3.8)

project (tpu_pulse_gen)

import_library (utils)
import_library (board)
import_library (net)
import_library (dev)
import_library (hash)
import_library (fs)
import_library (logging)
import_library (version_tag)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED 14)
set (CMAKE_CXX_EXTENSIONS OFF)

set (tpu_pulse_gen_romfs_files


  ${LIBRARIES_DIR}/webix/animate-css/animate.min.css		"animate-css/animate.css"		"text/css"
  ${LIBRARIES_DIR}/webix/slidebutton.css	"webix/slidebutton.css"		"text/css"

#  ${LIBRARIES_DIR}/webix/codebase/webix.min.css	"webix/codebase/webix.css"	"text/css"
  ${LIBRARIES_DIR}/webix/codebase/skins/mini.min.css "webix/codebase/skins/mini.css" "text/css"

  ${LIBRARIES_DIR}/webix/codebase/webix.min.js	"webix/codebase/webix.js"	"text/javascript"
  ${LIBRARIES_DIR}/webix/konva.js		"webix/konva.js"		"text/javascript"
  ${LIBRARIES_DIR}/webix/slidebutton.js		"webix/slidebutton.js"		"text/javascript"

#  ${LIBRARIES_DIR}/webix/codebase/fonts/fontawesome-webfont.woff "webix/codebase/fonts/fontawesome-webfont.woff" "font/woff"
  ${LIBRARIES_DIR}/webix/codebase/fonts/fontawesome-webfont.woff2 "webix/codebase/fonts/fontawesome-webfont.woff2" "font/woff2"

#  ${LIBRARIES_DIR}/webix/codebase/fonts/PTS-webfont.woff "webix/codebase/fonts/PTS-webfont.woff" "font/woff"

#  ${LIBRARIES_DIR}/webix/codebase/fonts/Roboto-Medium-webfont.woff "webix/codebase/fonts/Roboto-Medium-webfont.woff" "font/woff"
  ${LIBRARIES_DIR}/webix/codebase/fonts/Roboto-Medium-webfont.woff2 "webix/codebase/fonts/Roboto-Medium-webfont.woff2" "font/woff2"

#  ${LIBRARIES_DIR}/webix/codebase/fonts/Roboto-Regular-webfont.woff "webix/codebase/fonts/Roboto-Regular-webfont.woff" "font/woff"
  ${LIBRARIES_DIR}/webix/codebase/fonts/Roboto-Regular-webfont.woff2 "webix/codebase/fonts/Roboto-Regular-webfont.woff2" "font/woff2"

#  ${LIBRARIES_DIR}/webix/codebase/fonts/webixfa-webfont.woff "webix/codebase/fonts/webixfa-webfont.woff" "font/woff"
  ${LIBRARIES_DIR}/webix/codebase/fonts/webixfa-webfont.woff2 "webix/codebase/fonts/webixfa-webfont.woff2" "font/woff2"

#  ${LIBRARIES_DIR}/webix/codebase/fonts/webixmdi-webfont.woff "webix/codebase/fonts/webixmdi-webfont.woff" "font/woff"
  ${LIBRARIES_DIR}/webix/codebase/fonts/webixmdi-webfont.woff2 "webix/codebase/fonts/webixmdi-webfont.woff2" "font/woff2"

  ${LIBRARIES_DIR}/webix/font-awesome/web-fonts-with-css/css/fontawesome-all.min.css "fontawesome/css/all.css" "text/css"
#  ${LIBRARIES_DIR}/webix/font-awesome/web-fonts-with-css/webfonts/fa-brands-400.woff "fontawesome/webfonts/fa-brands-400.woff" "font/woff"
  ${LIBRARIES_DIR}/webix/font-awesome/web-fonts-with-css/webfonts/fa-brands-400.woff2 "fontawesome/webfonts/fa-brands-400.woff2" "font/woff2"
#  ${LIBRARIES_DIR}/webix/font-awesome/web-fonts-with-css/webfonts/fa-regular-400.woff "fontawesome/webfonts/fa-regular-400.woff" "font/woff"
  ${LIBRARIES_DIR}/webix/font-awesome/web-fonts-with-css/webfonts/fa-regular-400.woff2 "fontawesome/webfonts/fa-regular-400.woff2" "font/woff2"
#  ${LIBRARIES_DIR}/webix/font-awesome/web-fonts-with-css/webfonts/fa-solid-900.woff "fontawesome/webfonts/fa-solid-900.woff" "font/woff"
  ${LIBRARIES_DIR}/webix/font-awesome/web-fonts-with-css/webfonts/fa-solid-900.woff2 "fontawesome/webfonts/fa-solid-900.woff2" "font/woff2"

  ${LIBRARIES_DIR}/utils/utils.js		"utils/utils.js"		"text/javascript"

  ${LIBRARIES_DIR}/webix/konva/konva.min.js	"konva/konva.min.js"		"text/javascript"

  romfs_files/favicon.ico			favicon.ico			"image/x-icon"
  romfs_files/logo.svg				logo.svg			"image/svg+xml"
  index.html					index.html			"text/html"
)

add_target_executable (tpu_pulse_gen.elf)

target_sources (tpu_pulse_gen.elf
PRIVATE
  app_http_server.cpp
)

target_link_libraries (tpu_pulse_gen.elf
  dev
  utils
  net
  hash
  fs
  logging
  version_tag
)

add_romfs (tpu_pulse_gen.elf ${TARGET_ENDIAN} ${tpu_pulse_gen_romfs_files})

set_default_version_tag_product_name ("MCB TPU Pulse Gen")
set_default_version_tag_copyright_str ("Copyright (C) 2018, Jutze")


if (${CMAKE_SUBSYSTEM_NAME} MATCHES "EmbeddedMCU")

import_library (uip)
add_version_tag (tpu_pulse_gen.elf)

target_sources (tpu_pulse_gen.elf
PRIVATE
  main.cpp
)

target_link_libraries (tpu_pulse_gen.elf
  uip
)

target_link_library_group (tpu_pulse_gen.elf
  board board_crt stdc++ m c supc++ gcc
  utils
  dev
)

add_additional_executable_targets (tpu_pulse_gen.elf)

else ()

target_sources (tpu_pulse_gen.elf
PRIVATE
  main_httpserver.cpp
)

endif ()
