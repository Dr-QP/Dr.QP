cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_hexapod_js)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_custom_target(drqp_hexapod_js ALL
  COMMAND yarn
  # COMMAND yarn run generate-ros-messages
  COMMAND yarn build-prod

  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  USES_TERMINAL
  VERBATIM
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable auto copyright
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# install rules added by ros2pkg_configure_nodejs
install(FILES
  package.json
  yarn.lock
  DESTINATION share/${PROJECT_NAME}/dist
)

install(DIRECTORY
  packages
  DESTINATION share/${PROJECT_NAME}/dist
  PATTERN **/node_modules/** EXCLUDE
  OPTIONAL
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(CODE "execute_process(COMMAND yarn install WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/dist)")

