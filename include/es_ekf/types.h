// include/eskf/types.h
#pragma once

#include <Eigen/Dense>

struct PredictionResult{
    EigenVectorXd nominal_state;
    EigenVectorXd error_state;
    EigenVectorXd covariance;
}

struct UpdateResult{
    EigenVectorXd state;
    EigenVectorXd covariance;
    EigenVectorXd innovation;
    EigenVectorXd kalman_gain;
}