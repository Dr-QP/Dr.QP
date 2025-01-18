
include_directories(.)
add_executable(Relax Relax.cpp)
target_link_libraries(Relax PRIVATE drqp_a1_16_driver)

add_executable(ReadAll ReadAll.cpp)
target_link_libraries(ReadAll PRIVATE drqp_a1_16_driver)

add_executable(ReadEverything ReadEverything.cpp)
target_link_libraries(ReadEverything PRIVATE drqp_a1_16_driver)
