cmake_minimum_required(VERSION 2.8.3)
project(local_path_planning)

add_definitions (-std=c++11)

include (FindPkgConfig)
pkg_check_modules (${PROJECT_NAME}_extra REQUIRED eigen3)
include_directories (${${PROJECT_NAME}_extra_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/read.cc)
