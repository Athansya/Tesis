#include "LinearKalmanFilter.hpp"
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    double dt = 0.0;
    Eigen::MatrixXd F;
    Eigen::MatrixXd H;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;
    Eigen::MatrixXd B;

    LinearKalmanFilter lkf(dt, F, H, Q, R, P, B);

    return 0;
}