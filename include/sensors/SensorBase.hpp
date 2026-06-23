//include/es_ekf/sensprs/SensorBase.hpp
#pragma once

#include "core/State.hpp"
#include <Eigen/Dense>


class SensorBase{
    public:
        virtual ~SensorBase() = default;
        virtual Eigen::MatrixXd getNoiseCovariance() const = 0;
};