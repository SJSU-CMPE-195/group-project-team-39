cmake_minimum_required(VERSION 3.16)
project(cppsvc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(cppsvc main.cpp ${SOURCES})

target_include_directories(cppsvc PRIVATE include)

target_link_libraries(cppsvc PRIVATE gpiod)