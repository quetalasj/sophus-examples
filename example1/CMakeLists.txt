cmake_minimum_required(VERSION 3.12)
project(example1)

set(CMAKE_CXX_STANDARD 17)

find_package(Sophus REQUIRED)

#include_directories(${Sophus_INCLUDE_DIRS})
#message("Sophus_INCLUDE_DIRS = ${Sophus_INCLUDE_DIRS}")
include_directories("/usr/include/eigen3/")

add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus)
