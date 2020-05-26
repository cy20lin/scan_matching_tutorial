#include <iostream>
#include <Eigen/Dense>
#include <scan_matching/csv_reader.hpp>

int main() {
    Eigen::Matrix2Xd input_points;
    // readPointsFromCsv("../data/retangle_points10.csv", input_points);
    Eigen::Matrix2Xd rotated_points;
    // writePointsToCsv("../data/retangle_points10_rotated.csv", rotated_points);
    return 0;
}