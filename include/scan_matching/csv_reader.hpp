#include <string>
#include <Eigen/Dense>
bool readPointsFromCsv(std::string const & csv_file_path, Eigen::MatrixX2d &);
bool writePointsToCsv(std::string const & csv_file_path, Eigen::MatrixX2d const & points);