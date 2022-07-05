cmake_minimum_required(VERSION 3.21)
project(Dr_QP CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Werror)
add_compile_options(-fvisibility=hidden -fvisibility-inlines-hidden)

add_subdirectory(src)
add_subdirectory(tests)

