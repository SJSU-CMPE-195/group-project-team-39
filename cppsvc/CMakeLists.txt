cmake_minimum_required(VERSION 3.16)
project(cppsvc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SOURCES "src/*.cpp")

find_package(Threads REQUIRED)

add_executable(cppsvc main.cpp ${SOURCES})

target_include_directories(cppsvc PRIVATE include)

# rt provides shm_open / sem_open on Linux; macOS includes them in libc
if(UNIX AND NOT APPLE)
    target_link_libraries(cppsvc PRIVATE gpiod Threads::Threads rt)
else()
    target_link_libraries(cppsvc PRIVATE gpiod Threads::Threads)
endif()