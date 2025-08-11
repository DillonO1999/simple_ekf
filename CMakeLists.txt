cmake_minimum_required(VERSION 3.20)
project(simple_ekf)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(lcm REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_subdirectory(msgs)
add_subdirectory(src)
