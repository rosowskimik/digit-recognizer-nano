# License: Apache-2.0

cmake_minimum_required(VERSION 3.20)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(digit_recognizer LANGUAGES C CXX)

set(NO_THREADSAFE_STATICS $<TARGET_PROPERTY:compiler-cpp,no_threadsafe_statics>)
zephyr_compile_options($<$<COMPILE_LANGUAGE:CXX>:${NO_THREADSAFE_STATICS}>)

include_directories(include)

file(GLOB_RECURSE app_sources src/*.c
                              src/*.cpp)

target_sources(app PRIVATE ${app_sources})
