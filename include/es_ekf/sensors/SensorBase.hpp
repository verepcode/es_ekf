//include/es_ekf/sensprs/SensorBase.hpp
#pragma once

#include "es_ekf/core/State.hpp"
#include <Eigen/Dense>

namespace es_ekf{

class SensorBase{
    public:
        virtual ~SensorBase() = default;
        virtual Eigen::MatrixXd getNoiseCovariance() const = 0;
};
}//namespace es_ekf