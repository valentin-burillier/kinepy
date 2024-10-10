cmake_minimum_required(VERSION 3.28)
project(C_ified_kinepy C)

set(CMAKE_C_STANDARD 11)

add_executable(C_ified_kinepy
    src/system.h
    src/system.c
    src/graphs.h
    src/graphs.c
    src/test.c
)
