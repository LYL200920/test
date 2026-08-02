if(NOT DEFINED RUNTIME_DLLS OR RUNTIME_DLLS STREQUAL "")
  return()
endif()

string(REPLACE "|" ";" runtime_dll_list "${RUNTIME_DLLS}")
foreach(runtime_dll IN LISTS runtime_dll_list)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${runtime_dll}" "${DESTINATION}"
    COMMAND_ERROR_IS_FATAL ANY)
endforeach()
