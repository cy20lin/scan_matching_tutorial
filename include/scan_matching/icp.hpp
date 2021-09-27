#include <array>
#include <vector>
#include <Eigen/Dense>

void printIndicsPair(const std::vector<std::array<int, 2>> & pairs);

// Find nearest neighbor points in point cloud Q
// for every point in point cloud P
// return list of matching indices pairs.
// For each indices pair returned,
// the first index is the point index in point cloud P
// and the second index is the point index in point cloud Q
std::vector<std::array<int, 2>> nearestNeighborIndices(
    const Eigen::MatrixX3d &P, const Eigen::MatrixX3d &Q);

// Find the average squared error for all matching point pairs
double icpEvaluateError(
    const Eigen::MatrixX3d &P, const Eigen::MatrixX3d &Q, 
    const std::vector<std::array<int, 2>> & I
);