set(PROJECT_NAME drqp_hexapod_js)

execute_process(
  COMMAND ${CMAKE_CURRENT_LIST_DIR}/yarn_ws_install.sh ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/local_setup.bash
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/dist

  COMMAND_ERROR_IS_FATAL ANY
  TIMEOUT 300

  ECHO_OUTPUT_VARIABLE
  ECHO_ERROR_VARIABLE
  COMMAND_ECHO STDOUT
)

# For sequential execution of multiple commands use multiple execute_process calls each with a single COMMAND argument
execute_process(
  COMMAND find . -type d -name node_modules -exec touch {}/COLCON_IGNORE ";"
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/dist

  COMMAND_ERROR_IS_FATAL ANY
  TIMEOUT 300

  ECHO_OUTPUT_VARIABLE
  ECHO_ERROR_VARIABLE
  COMMAND_ECHO STDOUT
)
