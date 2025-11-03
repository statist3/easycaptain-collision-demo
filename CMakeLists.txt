cmake_minimum_required(VERSION 3.16)
project(cpa_risk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(cpa_risk
    src/main.cpp
    src/kalman.cpp
    src/cpa.cpp
    src/io.cpp
    src/radar.cpp
    src/json_writer.cpp
)

target_compile_features(cpa_risk PRIVATE cxx_std_17)

target_compile_options(cpa_risk PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)
