#include <string>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

// FIXME: how to write elegant and robust c++ code ?
bool readPointsFromCsv(std::string const & csv_file_path, Eigen::MatrixX2d & points) {
    std::ifstream file(csv_file_path);
    std::string line;
    std::string x, y;
    bool is_header_valid;
    is_header_valid = std::getline(file, line).operator bool();
    if (!is_header_valid) {
        return false;
    }
    std::stringstream ss(line);
    is_header_valid = std::getline(ss, x, ',') && std::getline(ss, y, ',') && (x == "x" && y == "y");
    if (!is_header_valid) {
        return false;
    }
    int i = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        if (std::getline(ss, x, ',') && std::getline(ss, y, ',')) {
            points.conservativeResize(i + 1, Eigen::NoChange);
            points(i, 0) = std::stod(x);
            points(i, 1) = std::stod(y);
            ++i;
        }
    }
    return true;
}

bool writePointsToCsv(std::string const & csv_file_path, Eigen::MatrixX2d const & points) {
    std::ofstream file(csv_file_path);
    file << "x,y" << std::endl;
    Eigen::Index column_size = points.cols();
    for (int column = 0; column < column_size; ++column) {
        double px = points(0, column);
        double py = points(1, column);
        file << px <<"," << py << std::endl;
    }
}