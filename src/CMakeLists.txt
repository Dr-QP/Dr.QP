
add_subdirectory(examples)
add_subdirectory(servo)

add_executable(DrQP main.cpp)

include_directories(.)
add_executable(Relax Relax.cpp)
target_link_libraries(Relax PRIVATE servo)

add_executable(ReadAll ReadAll.cpp)
target_link_libraries(ReadAll PRIVATE servo)