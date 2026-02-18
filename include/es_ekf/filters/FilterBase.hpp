//include/es_ekf/filters/FilterBase.hpp
#pragma once

#include "es_ekf/core/Types.hpp"
#include "es_ekf/core/State.hpp"

namespace es_ekf{

class FilterBase {
    public:
        virtual ~FilterBase() = default;
        virtual PredictionResult predict(const ImuData& imu, double dt) = 0;
        virtual UpdateResult update(const MeasurementData& meas) = 0;
        virtual const State& getState() const = 0;
        virtual const Eigen::MatrixXd& getCovariance() const = 0;

};
}//namespace es_ekf