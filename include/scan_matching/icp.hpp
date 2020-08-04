#include <array>
#include <vector>
#include <Eigen/Dense>

void printIndicsPair(const std::vector<std::array<int, 2>> & pairs);

// Find nearest neighbor points in point cloud Q
// for every point in point cloud P
// return list of matching indics pairs.
// For each indics pair returned,
// the first index is the point index in point cloud P
// and the second index is the point index in point cloud Q
std::vector<std::array<int, 2>> nearestNeighborIndics(
    const Eigen::MatrixX3d &P, const Eigen::MatrixX3d &Q);

