cmake_minimum_required(VERSION 3.10)
project(simple_ekf)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Python3 COMPONENTS Interpreter Development)
find_package(lcm REQUIRED)
include(${LCM_USE_FILE})
find_package(Java)

include_directories(include)

if(JAVA_FOUND)
  include(UseJava)
endif()

add_subdirectory(msgs)
add_subdirectory(src)
