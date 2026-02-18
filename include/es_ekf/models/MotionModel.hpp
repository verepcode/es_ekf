//include/es_ekf/models/MotionModel.hpp

#pragma once

#include "es_ekf/core/State.hpp"
#include "es_ekf/core/Types.hpp"
#include <Eigen/Dense>

namespace es_ekf{

class MotionModel{
    public:
        virtual ~MotionModel() = default;
        virtual State propagate(const State& state, const ImuData& imu, double dt) = 0;
        virtual Eigen::MatrixXd getJacobian(const State& state, double dt) = 0;
        virtual Eigen::MatrixXd getNoiseCovariance(double dt) = 0;
};
}//namespace es_ekf