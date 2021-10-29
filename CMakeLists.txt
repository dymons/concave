cmake_minimum_required(VERSION 3.5.1)

if(POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif(POLICY CMP0048)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(concave)

option(CONCAVE_DEVELOPER "Concave development mode" OFF)

add_subdirectory(concave_core)

if(CONCAVE_DEVELOPER)
    add_subdirectory(concave_test)
endif()
