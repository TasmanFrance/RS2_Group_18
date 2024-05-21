cmake_minimum_required(VERSION 3.5.0)
project(cpp VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

add_executable(cpp publisher.cpp graph.cpp greedy.cpp processor.cpp) #asdf.h

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
