cmake_minimum_required(VERSION 3.1)

project(scan_matching)

find_package(Eigen3 REQUIRED NO_MODULE)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(scan_matching_libs src/csv_reader.cpp src/icp.cpp)

add_executable(01_eigen_tester apps/01_eigen_tester.cpp)
add_executable(02_csv_points_rotation apps/02_csv_points_rotation.cpp)
target_link_libraries(02_csv_points_rotation scan_matching_libs)

add_executable(03_icp_nearest_neighbor apps/03_icp_nearest_neighbor.cpp)
target_link_libraries(03_icp_nearest_neighbor scan_matching_libs)

add_executable(04_icp_evaluate_error apps/04_icp_evaluate_error.cpp)
target_link_libraries(04_icp_evaluate_error scan_matching_libs)

add_executable(00_icp apps/00_icp.cpp)
target_link_libraries(00_icp scan_matching_libs Eigen3::Eigen)
