cmake_minimum_required(VERSION 2.8)
project(concave_test)

find_package(Boost 1.65.1 COMPONENTS filesystem REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)

include(${CGAL_USE_FILE})
add_subdirectory("/usr/src/gtest" ${CMAKE_BINARY_DIR}/gtest)
enable_testing()
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(concave_convexhull_tests concave_convexhull_tests.cpp)
add_to_cached_list(CGAL_EXECUTABLE_TARGETS concave_convexhull_tests)
target_link_libraries(concave_convexhull_tests
                      gtest
                      concave::concave_core
                      ${Boost_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${CGAL_LIBRARIES}
                      ${CGAL_3RD_PARTY_LIBRARIES}
                      )
add_test(NAME concave_convexhull_tests COMMAND concave_convexhull_tests)

add_executable(concave_utility_tests concave_utility_tests.cpp)
add_to_cached_list(CGAL_EXECUTABLE_TARGETS concave_utility_tests)
target_link_libraries(concave_utility_tests
                      gtest
                      concave::concave_core
                      ${OpenCV_LIBS}
                      ${CGAL_LIBRARIES}
                      ${CGAL_3RD_PARTY_LIBRARIES}
                      )
add_test(NAME concave_utility_tests COMMAND concave_utility_tests)