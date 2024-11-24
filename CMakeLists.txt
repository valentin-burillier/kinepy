cmake_minimum_required(VERSION 3.28)
project(C_ified_kinepy C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

add_library(
        C_Kinepy
        STATIC
        kinepy_src/graph/graph_operations.c
        kinepy_src/graph/graph_data.c
        kinepy_src/interface/interface.c
        kinepy_src/interface/configuration.c
        kinepy_src/interface/helper_functions.c
        kinepy_src/kinematics/kinematics.h
        kinepy_src/kinematics/kinematics.c
)
set_target_properties(C_Kinepy PROPERTIES LINKER_LANGUAGE C)
target_include_directories(C_Kinepy PUBLIC kinepy_src)

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

set_target_properties(unit_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/unit_tests")


add_executable(test_unique
    kinepy_src/test.c
)
target_compile_options(
        C_Kinepy PUBLIC
        -mavx2
        -mfma
)

target_compile_options(
        test_unique PUBLIC
        -I${CMAKE_CURRENT_SOURCE_DIR}/kinepy_src
)