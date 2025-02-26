execute_process(
  COMMAND yarn install
  COMMAND find . -fstype dir -name node_modules -exec touch {}/COLCON_IGNORE ";"
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/dist

  COMMAND_ERROR_IS_FATAL ANY
  COMMAND_ECHO STDOUT
  TIMEOUT 300
)
