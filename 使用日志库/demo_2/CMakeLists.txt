cmake_minimum_required(VERSION 3.16)
project(log_stage2 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
add_executable(log_stage2 main.cpp)
target_include_directories(log_stage2 PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/..")
target_link_libraries(log_stage2 PRIVATE Threads::Threads)
