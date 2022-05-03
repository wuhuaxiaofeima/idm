# CMakeLists.txt: RDBHandler
cmake_minimum_required (VERSION 3.0)

# compiler options
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set (CMAKE_BUILD_TYPE "Release")
set (CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set (CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# compile library
add_library(rdb_handler "RDBHandler.cc")