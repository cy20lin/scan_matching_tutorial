#include <iostream>
#include <Eigen/Dense>

int main(int argc, char const ** argv) {
    Eigen::Vector2d a(5.0, 6.0);
    Eigen::Vector2d b(5.0, 6.0);
    std::cout << (a + b).transpose() << std::endl;
    return 0;
}