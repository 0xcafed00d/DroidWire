cmake_minimum_required(VERSION 3.16)
project(droidwire
    VERSION 1.0.0
    DESCRIPTION "Cross-platform C++ serial port library"
    LANGUAGES CXX
)

# --- Options ---
option(DROIDWIRE_BUILD_TESTS "Build test executable" OFF)

# --- Library target ---
add_library(droidwire STATIC
    include/droidwire/droidwire.h
    src/droidwire.cpp
)

# --- Include directories ---
target_include_directories(droidwire
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# --- Set language standard and warnings ---
target_compile_features(droidwire PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(droidwire PRIVATE /W4 /permissive-)
else()
    target_compile_options(droidwire PRIVATE -Wall -Wextra -Wpedantic)
endif()

# --- Platform-specific links if needed ---
if(WIN32)
    target_link_libraries(droidwire PRIVATE setupapi)
endif()

