#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <array>
#include <cmath>
#include <scan_matching/csv_reader.hpp>
#include <iostream>
#include <scan_matching/icp.hpp>
#include <string>

using point_cloud_type = Eigen::MatrixX3d;
using index_pair_type = std::array<int, 2>;
using indics_type = std::vector<index_pair_type>;

struct transform {
  transform() : theta(0), tx(0), ty(0) {}
  double theta;
  double tx;
  double ty;
};

std::ostream & operator <<(std::ostream &os, transform T) {
    return os << "{"
        "\"theta\": " << T.theta << ", "
        "\"tx\": " << T.tx << ", "
        "\"ty\": " << T.ty << "}";
}

using transform_type = transform;

indics_type matching(point_cloud_type P, point_cloud_type Q);
transform_type align(point_cloud_type P, point_cloud_type Q, indics_type I);
point_cloud_type apply_transform(point_cloud_type P, transform_type T);
transform_type apply_transform(transform_type T0, transform_type T);
double evaluate_error(point_cloud_type P, point_cloud_type Q, indics_type I);

indics_type matching(point_cloud_type P, point_cloud_type Q) {
  std::vector<std::array<int, 2>> result;
  std::array<int, 2> pair;
  int N = P.rows();
  int M = Q.rows();
  int min_j = -1;
  for (int i = 0; i < N; ++i) {
    double min_distance2 = std::numeric_limits<double>().infinity();
    for (int j = 0; j < M; ++j) {
      double dx = P(i, 0) - Q(j, 0);
      double dy = P(i, 1) - Q(j, 1);
      double distance2 = dx * dx + dy * dy;
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

double evaluate_error(point_cloud_type P, point_cloud_type Q, indics_type I) {
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

transform_type align(point_cloud_type P, point_cloud_type Q, indics_type I) {
  double mean_px = 0;
  double mean_py = 0;
  double mean_qx = 0;
  double mean_qy = 0;
  for (auto pair : I) {
    auto i = pair[0];
    auto j = pair[1];
    double px = P(i, 0);
    double py = P(i, 1);
    mean_px += px;
    mean_py += py;
    double qx = Q(j, 0);
    double qy = Q(j, 1);
    mean_qx += qx;
    mean_qy += qy;
  }
  mean_px /= I.size();
  mean_py /= I.size();
  mean_qx /= I.size();
  mean_qy /= I.size();
  Eigen::MatrixX2d P_(I.size(), 2);
  Eigen::MatrixX2d Q_(I.size(), 2);

  for (auto pair : I) {
    auto i = pair[0];
    auto j = pair[1];
    P_(i, 0) = P(i, 0) - mean_px;
    P_(i, 1) = P(i, 1) - mean_py;
    Q_(j, 0) = Q(j, 0) - mean_qx;
    Q_(j, 1) = Q(j, 1) - mean_qy;
  }
  Eigen::Matrix2d W = Q_.transpose() * P_;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      W, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix2d V = svd.matrixV();
  Eigen::Matrix2d U = svd.matrixU();
  Eigen::Matrix2d R = U * V.transpose();
  Eigen::Vector2d mean_p;
  mean_p(0) = mean_px;
  mean_p(1) = mean_py;
  Eigen::Vector2d mean_q;
  mean_q(0) = mean_qx;
  mean_q(1) = mean_qy;
  Eigen::Vector2d t = mean_q - R * mean_p;
  double cos2 = R(0, 0) + R(1, 1);
  double sin2 = -R(0, 1) + R(1, 0);
  double theta = std::atan2(sin2, cos2);
  double tx = t(0);
  double ty = t(1);
  transform_type T;
  T.theta = theta;
  T.tx = tx;
  T.ty = ty;
  return T;
}

point_cloud_type apply_transform(point_cloud_type P, transform_type T) {
  Eigen::Matrix3d T_;
  double s = std::sin(T.theta);
  double c = std::cos(T.theta);
  T_ << 
    c, -s, T.tx,
    s, c, T.ty,
    0, 0, 1;
  return (T_ * P.transpose()).transpose();
}

transform_type apply_transform(transform_type T0, transform_type T) {
  Eigen::Matrix3d T0_;
  Eigen::Matrix3d T_;
  double s0 = std::sin(T0.theta);
  double c0 = std::cos(T0.theta);
  T_ << 
    c0, -s0, T0.tx,
    s0, c0, T0.ty,
    0, 0, 1;
  double s = std::sin(T.theta);
  double c = std::cos(T.theta);
  T_ << 
    c, -s, T.tx,
    s, c, T.ty,
    0, 0, 1;

  transform_type T2;
  Eigen::Matrix3d T2_ = T_ * T0_;
  double c2 = T2_(0,0) + T2_(1,1);
  double s2 = - T2_(0,1) + T2_(1,0);
  T2.tx = T2_(0,2);
  T2.ty = T2_(1,2);
  T2.theta = std::atan2(s2, c2);
  return T2;
}

int main(int argc, char**argv) { 

    double theta_ = 0.785;
    double tx_ = 0.5;
    double ty_ = 0.5;
    int max_iteration_ = 1;
    transform_type real_T;
    int max_iteration = argc > 1 ? std::atoi(argv[1]) : max_iteration_;
    real_T.theta = argc > 2 ? std::atof(argv[2]) : theta_;
    real_T.tx = argc > 3 ? std::atof(argv[3]) : tx_;
    real_T.ty = argc > 4 ? std::atof(argv[4]) : ty_;

    std::cout << "Parameters:" << std::endl;
    std::cout << real_T;
    std::cout << std::endl;

    Eigen::MatrixX3d P;
    readPointsFromCsv("../data/rectangle_points10.csv", P);

    std::cout << "Input points:" << std::endl;
    std::cout << P << std::endl;
    std::cout << std::endl;
    Eigen::MatrixX3d Q;
    Q = apply_transform(P, real_T);
    writePointsToCsv("../data/rectangle_points10_rotated.csv", Q);
    std::cout << "Rotated points:" << std::endl;
    std::cout << Q << std::endl;
    std::cout << std::endl;

    int iteration = 0;
    double error = std::numeric_limits<double>().infinity();
    point_cloud_type P_ = P;
    transform_type T;
    transform_type T_;
    indics_type I;
    double pre_error, post_error;
    writePointsToCsv(std::string{} + "../data/icp_iteration_0.csv", P);
    while (iteration < max_iteration) {
        I = matching(P_, Q);
        pre_error = evaluate_error(P_, Q, I);
        T_ = align(P_, Q, I);
        P_ = apply_transform(P_, T_);
        writePointsToCsv(std::string{} + "../data/icp_iteration_" + std::to_string(iteration + 1) + ".csv", P_);
        T = apply_transform(T, T_);
        post_error = evaluate_error(P_, Q, I);
        std::cout << "iteration " << iteration << std::endl
          << "  transform  = "<< T << std::endl
          << "  transform_ = "<< T_ << std::endl
          << "  pre_error  = "<< pre_error << std::endl
          << "  post_error = "<< post_error << std::endl;
        ++iteration;
    }
    return 0;
}