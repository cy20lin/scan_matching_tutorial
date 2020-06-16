#include <iostream>
#include <Eigen/Dense>
#include <scan_matching/csv_reader.hpp>


int main(int argc, char **argv) {
    // argv[1]: theta
    // argv[2]: tx
    // argv[3]: ty
    double theta = argc > 1 ? std::atof(argv[1]) : 0;
    double tx = argc > 2 ? std::atof(argv[2]) : 0;
    double ty = argc > 3 ? std::atof(argv[3]) : 0;
    Eigen::Matrix3d transform;
    transform <<
        std::cos(theta), -std::sin(theta), tx,
        std::sin(theta), std::cos(theta), ty,
        0, 0, 1;
    Eigen::Rotation2D<double> rotation(theta);
    Eigen::Translation<double,2> translation(tx, ty);

    std::cout << "theta: " << theta << std::endl;
    std::cout << "tx: " << tx << std::endl;
    std::cout << "ty: " << ty << std::endl;

    Eigen::MatrixX3d input_points;
    readPointsFromCsv("../data/rectangle_points10.csv", input_points);

    std::cout << input_points << std::endl;
    Eigen::MatrixX3d rotated_points = (transform * input_points.transpose()).transpose();
    writePointsToCsv("../data/rectangle_points10_rotated.csv", rotated_points);
    std::cout << rotated_points << std::endl;
    return 0;
}