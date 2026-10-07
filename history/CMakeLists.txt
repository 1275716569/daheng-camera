cmake_minimum_required(VERSION 3.16)
project(OpenCVDemo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(opencv_demo main.cpp)
target_include_directories(opencv_demo PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(opencv_demo PRIVATE ${OpenCV_LIBS})

add_executable(my_daheng my_daheng.cpp)
target_include_directories(my_daheng PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/opencv_demo/include"
    ${OpenCV_INCLUDE_DIRS})
target_link_libraries(my_daheng ${OpenCV_LIBS} gxiapi)
