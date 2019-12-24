cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
project(concave VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS} -D_GLIBCXX_DEBUG -fsanitize=address -fsanitize=undefined")

add_subdirectory(concave_core)
add_subdirectory(concave_test)