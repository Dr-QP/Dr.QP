execute_process(
  COMMAND yarn install
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/dist

  COMMAND_ERROR_IS_FATAL ANY
  COMMAND_ECHO STDOUT
  TIMEOUT 300
)

# For sequential execution of multiple commands use multiple execute_process calls each with a single COMMAND argument
execute_process(
  COMMAND find . -type d -name node_modules -exec touch {}/COLCON_IGNORE ";"
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/dist

  COMMAND_ERROR_IS_FATAL ANY
  COMMAND_ECHO STDOUT
  TIMEOUT 300
)
