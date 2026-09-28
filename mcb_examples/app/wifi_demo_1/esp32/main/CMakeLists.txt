set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED 14)
set (CMAKE_CXX__EXTENSIONS OFF)

set(COMPONENT_SRCS "main.cpp" "websocket_server.c" "websocket.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_EMBED_FILES "./server/index.html" "./server/error.html" "./server/test.js" "./server/test.css" "./server/favicon.ico")

register_component()

import_library (utils)
import_library (hash)
target_link_libraries (main utils hash)
