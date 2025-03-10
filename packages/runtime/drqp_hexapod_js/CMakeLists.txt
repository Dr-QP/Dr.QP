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

if (DRQP_SKIP_WEB)
  set_target_properties(drqp_hexapod_js PROPERTIES EXCLUDE_FROM_ALL TRUE)
endif()

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

  function(add_yarn_jest_test TARGET)
    if (DRQP_SKIP_WEB)
      return()
    endif()

    cmake_parse_arguments(ARG
      ""
      "RUN_CMD;RESULT_FILE;OUTPUT_FILE;WORKING_DIRECTORY"
      "EXTRA_COMMAND"
      ${ARGN})

    if(ARG_RUN_CMD)
      set(RUN_CMD "${ARG_RUN_CMD}")
    else()
      set(RUN_CMD "$<TARGET_FILE:${TARGET}>")
    endif()

    if(ARG_RESULT_FILE)
      set(RESULT_FILE "${ARG_RESULT_FILE}")
    else()
      set(RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TARGET}-jest.xunit.xml")
    endif()

    set(DEFAULT_OUT_DIR ${CMAKE_BINARY_DIR}/jest_test_results/${TARGET})
    if(ARG_OUTPUT_FILE)
      set(OUTPUT_FILE "${ARG_OUTPUT_FILE}")
    else()
      set(OUTPUT_FILE "${DEFAULT_OUT_DIR}/output.txt.ans")
    endif()

    set(CMD
      yarn run ${RUN_CMD}
      --reporters="jest-junit"
      --reporters="default"
      --coverage
      --coverageReporters lcov
      --coverageDirectory "${DEFAULT_OUT_DIR}/coverage/"
      ${ARG_EXTRA_COMMAND}
    )

    ament_add_test(
      ${TARGET}
      COMMAND ${CMD}
      WORKING_DIRECTORY ${ARG_WORKING_DIRECTORY}
      RESULT_FILE ${RESULT_FILE}
      OUTPUT_FILE ${OUTPUT_FILE}
      ENV JEST_JUNIT_OUTPUT_FILE=${RESULT_FILE}
    )
  endfunction()

  add_yarn_jest_test(
    hexapod-server
    RUN_CMD test-server
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/packages/hexapod
    EXTRA_COMMAND --reporters="github-actions"
  )
  add_yarn_jest_test(
    hexapod-client
    RUN_CMD test-client
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/packages/hexapod
  )
  add_yarn_jest_test(
    hexapod-kinematics-library
    RUN_CMD test
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/packages/hexapod-kinematics-library
    EXTRA_COMMAND --reporters="github-actions"
  )
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


