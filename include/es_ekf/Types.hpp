// include/es_ekf/types.hpp
#pragma once

#include <Eigen/Dense>

struct PredictionResult{
    Eigen::VectorXd nominal_state;
    Eigen::VectorXd error_state;
    Eigen::VectorXd covariance;
};

struct UpdateResult{
    Eigen::VectorXd state;
    Eigen::VectorXd covariance;
    Eigen::VectorXd innovation;
    Eigen::VectorXd kalman_gain;
};