cmake_minimum_required(VERSION 3.10)
project(Vector3DProject LANGUAGES CXX)

find_package(GTest REQUIRED)

add_library(Vector3D src/Vector3D.cpp)
target_include_directories(Vector3D PUBLIC include)

enable_testing()
add_executable(tests test/tests.cpp src/Vector3D.cpp)
target_link_libraries(tests Vector3D GTest::gtest GTest::gtest_main)

add_test(NAME AllTests COMMAND tests)