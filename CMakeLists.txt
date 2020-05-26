cmake_minimum_required(VERSION 3.1)

project(scan_matching)

find_package(Eigen3 REQUIRED NO_MODULE)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(scan_matching_libs src/csv_reader.cpp)

add_executable(01_eigen_tester apps/01_eigen_tester.cpp)
add_executable(02_csv_points_rotation apps/02_csv_points_rotation.cpp)