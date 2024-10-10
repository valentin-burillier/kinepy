cmake_minimum_required(VERSION 3.28)
project(C_ified_kinepy C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

add_library(
        C_Kinepy
        STATIC
        kinepy_src/system.h
        kinepy_src/system.c
        kinepy_src/graphs.h
        kinepy_src/graphs.c
)
set_target_properties(C_Kinepy PROPERTIES LINKER_LANGUAGE C)


include(FetchContent)
enable_testing()
#
# Include GoogleTest
#
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(unit_tests
        tests/test.cpp
)

target_link_libraries(unit_tests GTest::gtest_main C_Kinepy)
set_target_properties(unit_tests PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(unit_tests PUBLIC kinepy_src)
