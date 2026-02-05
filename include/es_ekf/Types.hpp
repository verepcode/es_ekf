// include/es_ekf/types.hpp
#pragma once

#include <Eigen/Dense>

namespace es_ekf{

    struct ImuData
    {
        Eigen::Vector3d linear_accel;
        Eigen::Vector3d ang_vel;
        double timestamp;
    };

    struct GNSSData
    {
        Eigen::Vector3d position;
        Eigen::Vector3d covariance;
        double timestamp;
    };

    struct PredictionResult
    {
        Eigen::VectorXd nominal_state;
        Eigen::VectorXd error_state;
        Eigen::VectorXd covariance;
    };

    struct UpdateResult
    {
        Eigen::VectorXd state;
        Eigen::VectorXd covariance;
        Eigen::VectorXd innovation;
        Eigen::VectorXd kalman_gain;
    };
}
