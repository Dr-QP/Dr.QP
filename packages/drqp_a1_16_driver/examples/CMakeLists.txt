
add_executable(read_everything ReadEverything.cpp)
target_link_libraries(read_everything PRIVATE drqp_a1_16_driver)

# Install executables for `ros2 run`
install(TARGETS
  read_everything
  DESTINATION lib/${PROJECT_NAME})
