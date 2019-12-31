cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
project(concave_test VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS} -D_GLIBCXX_DEBUG -fsanitize=address -fsanitize=undefined --coverage -fconcepts")

find_package(Boost 1.65.1 COMPONENTS filesystem REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)

include(${CGAL_USE_FILE})

add_subdirectory(google-test)

add_executable(concave_convexhull_tests concave_convexhull_tests.cpp)
add_to_cached_list(CGAL_EXECUTABLE_TARGETS concave_convexhull_tests)
target_link_libraries(concave_convexhull_tests
  concave::concave_core
  GTest::GTest
  GTest::Main
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${CGAL_LIBRARIES}
  ${CGAL_3RD_PARTY_LIBRARIES}
)
add_test(NAME concave_convexhull_tests COMMAND concave_convexhull_tests)

add_executable(concave_utility_tests concave_utility_tests.cpp)
add_to_cached_list(CGAL_EXECUTABLE_TARGETS concave_utility_tests)

target_link_libraries(concave_utility_tests
  concave::concave_core
  GTest::GTest
  GTest::Main
  ${OpenCV_LIBS}
  ${CGAL_LIBRARIES}
  ${CGAL_3RD_PARTY_LIBRARIES}
)

add_test(NAME concave_utility_tests COMMAND concave_utility_tests)