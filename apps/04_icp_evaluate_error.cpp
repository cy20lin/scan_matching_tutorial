#include <iostream>
#include <Eigen/Dense>
#include <scan_matching/csv_reader.hpp>
#include <scan_matching/icp.hpp>


int main(int argc, char **argv) {
    // argv[1]: theta
    // argv[2]: tx
    // argv[3]: ty
    double theta_ = 1.57 / 2;
    double tx_ = 0.5;
    double ty_ = 0.5;
    double theta = argc > 1 ? std::atof(argv[1]) : theta_;
    double tx = argc > 2 ? std::atof(argv[2]) : tx_;
    double ty = argc > 3 ? std::atof(argv[3]) : ty_;
    Eigen::Matrix3d transform;
    transform <<
        std::cos(theta), -std::sin(theta), tx,
        std::sin(theta), std::cos(theta), ty,
        0, 0, 1;
    Eigen::Rotation2D<double> rotation(theta);
    Eigen::Translation<double,2> translation(tx, ty);


    std::cout << "Parameters:" << std::endl;
    std::cout << "theta: " << theta << std::endl;
    std::cout << "tx: " << tx << std::endl;
    std::cout << "ty: " << ty << std::endl;
    std::cout << std::endl;

    Eigen::MatrixX3d input_points;
    readPointsFromCsv("../data/rectangle_points10.csv", input_points);

    std::cout << "Input points:" << std::endl;
    std::cout << input_points << std::endl;
    std::cout << std::endl;
    Eigen::MatrixX3d rotated_points = (transform * input_points.transpose()).transpose();
    writePointsToCsv("../data/rectangle_points10_rotated.csv", rotated_points);
    std::cout << "Rotated points:" << std::endl;
    std::cout << rotated_points << std::endl;
    std::cout << std::endl;

    auto & P = input_points;
    auto & Q = rotated_points;
    {
        std::cout << "icpEvaluateError(P, P, I) = " ;
        auto pairs = nearestNeighborIndics(P, P);
        double error = icpEvaluateError(P, P, pairs);
        std::cout << error << std::endl;
    }
    {
        std::cout << "icpEvaluateError(Q, Q, I) = " ;
        auto pairs = nearestNeighborIndics(Q, Q);
        double error = icpEvaluateError(Q, Q, pairs);
        std::cout << error << std::endl;
    }
    {
        std::cout << "icpEvaluateError(P, Q, I) = " ;
        auto pairs = nearestNeighborIndics(P, Q);
        double error = icpEvaluateError(P, Q, pairs);
        std::cout << error << std::endl;
    }
    return 0;
}