//include/es_ekf/sensprs/SensorBase.hpp
#pragma once

#include "es_ekf/core/State.hpp"
#include <Eigen/Dense>

namespace es_ekf{

class SensorBase{
    public:
        virtual ~SensorBase() = default;
        virtual Eigen::VectorXd predict(const State& state) = 0;
        virtual Eigen::MatrixXd getJacobian(const State& state) = 0;
        virtual Eigen::MatrixXd getNoiseCovariance() = 0;
};
}//namespace es_ekf