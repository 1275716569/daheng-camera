cmake_minimum_required(VERSION 3.16)
project(log_stage3 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
add_executable(log_stage3 main.cpp
    "${CMAKE_CURRENT_LIST_DIR}/../util/rm_log/src/rm_log.cpp")
target_include_directories(log_stage3 PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../util"
    "${CMAKE_CURRENT_LIST_DIR}/../util/rm_log/inlcude")
target_link_libraries(log_stage3 PRIVATE Threads::Threads)
