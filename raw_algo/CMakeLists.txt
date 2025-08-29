cmake_minimum_required(VERSION 4.0)
project(raw_algo)

set(CMAKE_CXX_STANDARD 20)

add_executable(raw_algo main.cpp
                mms-cpp/API.h
                mms-cpp/API.cpp
)
include_directories(mms-cpp)