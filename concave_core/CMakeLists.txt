cmake_minimum_required(VERSION 3.5.1)
project(concave_core)

add_library(${PROJECT_NAME} INTERFACE)
add_library(concave::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE ${PROJECT_SOURCE_DIR}/include)
