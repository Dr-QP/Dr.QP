
add_executable(read_everything ReadEverything.cpp)
target_link_libraries(read_everything PRIVATE drqp_a1_16_driver)

# Install libraries
install(TARGETS
  read_everything
  RUNTIME DESTINATION bin
)
