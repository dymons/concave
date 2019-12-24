cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
project(concave_core VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} INTERFACE)
add_library(concave::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  INTERFACE
    ${PROJECT_SOURCE_DIR}/include
)