cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_hexapod_js)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_custom_target(drqp_hexapod_js ALL
  COMMAND yarn install
  COMMAND find . -fstype dir -name node_modules -exec touch {}/COLCON_IGNORE {}/AMENT_IGNORE ";"
  # COMMAND yarn run generate-ros-messages
  COMMAND yarn build-prod

  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  USES_TERMINAL
  VERBATIM
)

if(BUILD_TESTING)
  ament_package_xml()
  foreach(dep ${${PROJECT_NAME}_TEST_DEPENDS})
    find_package(${dep} REQUIRED)
  endforeach()

  set(NODE_MODULES
    node_modules/
    packages/hexapod/node_modules/
    packages/hexapod-kinematics-library/node_modules/
  )

  ament_flake8(${CMAKE_CURRENT_LIST_DIR}/launch)
  ament_pep257(${CMAKE_CURRENT_LIST_DIR}/launch)

  # Skip for now as there is no easy way to configure excluded folders
  # ament_copyright(EXCLUDE ${NODE_MODULES})
  # ament_xmllint(EXCLUDE ${NODE_MODULES})
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
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(SCRIPT scripts/ExecYarnInstall.cmake)


