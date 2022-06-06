cmake_minimum_required(VERSION 3.12)
project(example1)

set(CMAKE_CXX_STANDARD 17)

find_package(Sophus REQUIRED)

include_directories("/usr/include/eigen3/")
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError Sophus::Sophus ${Pangolin_LIBRARIES})
