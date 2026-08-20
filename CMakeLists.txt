cmake_minimum_required(VERSION 3.20)

project(datacache LANGUAGES CXX)

add_executable(datacache src/main.cc)
target_include_directories(datacache PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(datacache PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(datacache PRIVATE /W4 /permissive-)
else()
    target_compile_options(datacache PRIVATE -Wall -Wextra -Wpedantic)
endif()
