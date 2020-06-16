#include <string>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

// FIXME: how to write elegant and robust c++ code ?
bool readPointsFromCsv(std::string const & csv_file_path, Eigen::MatrixX3d & points) {
    std::ifstream file(csv_file_path);
    std::string line;
    std::string x, y;
    bool is_header_valid;
    int i = 0;
    std::getline(file, line);
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        if (std::getline(ss, x, ',') && std::getline(ss, y, ',')) {
            points.conservativeResize(i + 1, Eigen::NoChange);
            points(i, 0) = std::stod(x);
            points(i, 1) = std::stod(y);
            points(i, 2) = 1;
            ++i;
        }
    }
    return true;
}

bool writePointsToCsv(std::string const & csv_file_path, Eigen::MatrixX3d const & points) {
    std::ofstream file(csv_file_path);
    file << "x,y" << std::endl;
    Eigen::Index row_size = points.rows();
    for (int row = 0; row <row_size; ++row) {
        double px = points(row, 0);
        double py = points(row, 1);
        file << px <<"," << py << std::endl;
    }
}