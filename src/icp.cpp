#include <scan_matching/icp.hpp>
#include <limits>
#include <iostream>

void printIndicsPair(const std::vector<std::array<int, 2>> & pairs) {
    for (const auto & pair : pairs) {
        std::cout << "(" << pair[0] << ", " << pair[1] << ")"<< std::endl;
    }
}

std::vector<std::array<int, 2>> nearestNeighborIndics( const Eigen::MatrixX3d &P, const Eigen::MatrixX3d &Q)
{ 
    std::vector<std::array<int, 2>> result;
    std::array<int, 2> pair;
    int N = P.rows();
    int M = Q.rows();
    int min_j = -1;
    for (int i = 0; i < N; ++i) {
        double min_distance2 = std::numeric_limits<double>().infinity();
        for (int j = 0; j < M; ++j) {
            double dx = P(i,0) - Q(j,0);
            double dy = P(i,1) - Q(j,1);
            double distance2 = dx*dx + dy*dy;
            if (distance2 < min_distance2) {
                min_distance2 = distance2;
                min_j = j;
            }
        }
        pair[0] = i;
        pair[1] = min_j;
        result.push_back(pair);
    }
    return result;
}

double icpEvaluateError(
    const Eigen::MatrixX3d &P, const Eigen::MatrixX3d &Q, 
    const std::vector<std::array<int, 2>> & I
) {
  double error = 0;
  if (!I.size())
    return 0;
  for (auto pair : I) {
    auto i = pair[0];
    auto j = pair[1];
    double px = P(i, 0);
    double py = P(i, 1);
    double qx = Q(j, 0);
    double qy = Q(j, 1);
    double dx = px - qx;
    double dy = py - qy;
    double distance_square = dx * dx + dy * dy;
    error += distance_square;
  }
  error /= I.size();
  return error;
}
