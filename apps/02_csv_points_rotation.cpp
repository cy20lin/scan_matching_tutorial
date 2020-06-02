#include <iostream>
#include <Eigen/Dense>
#include <scan_matching/csv_reader.hpp>

int main() {
    Eigen::MatrixX2d input_points;
    readPointsFromCsv("../data/rectangle_points10.csv", input_points);
    std::cout << input_points << std::endl;
    Eigen::MatrixX2d rotated_points = input_points;
    writePointsToCsv("../data/rectangle_points10_rotated.csv", rotated_points);
    return 0;
}