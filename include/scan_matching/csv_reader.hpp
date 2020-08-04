#include <Eigen/Dense>
#include <string>
bool readPointsFromCsv(std::string const &csv_file_path, Eigen::MatrixX3d &);
bool writePointsToCsv(std::string const &csv_file_path,
                      Eigen::MatrixX3d const &points);
