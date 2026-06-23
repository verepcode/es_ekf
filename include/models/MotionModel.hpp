//include/es_ekf/models/MotionModel.hpp

#pragma once

#include "core/State.hpp"
#include "core/Types.hpp"
#include <Eigen/Dense>

class MotionModel{
    public:
        virtual ~MotionModel() = default;
        virtual State propagate(const State& state, const ImuData& imu, double dt) = 0;
        virtual Eigen::MatrixXd getJacobian(const State& state, double dt) = 0;
        virtual Eigen::MatrixXd getNoiseCovariance(double dt) = 0;
};