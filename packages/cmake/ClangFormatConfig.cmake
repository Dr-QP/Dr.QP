
if(BUILD_TESTING)
  set(CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/../../.clang-format")
  get_filename_component(CONFIG_FILE_RES "${CONFIG_FILE}" REALPATH)

  if(EXISTS ${CONFIG_FILE_RES})
    set(ament_cmake_clang_format_CONFIG_FILE "${CONFIG_FILE_RES}" CACHE STRING "Path to the clang-format file to use")
  else()
    message(FATAL_ERROR ".clang-format config \"${CONFIG_FILE_RES}\" does not exist")
  endif()
endif()
