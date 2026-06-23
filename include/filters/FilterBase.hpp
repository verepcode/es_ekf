//include/filters/FilterBase.hpp
#pragma once

#include "core/Types.hpp"
#include "core/State.hpp"

class FilterBase {
    public:
        virtual ~FilterBase() = default;
        virtual State predict(const ImuData& imu, double dt) = 0;
        virtual State update(const MeasurementData& meas, State& state_check) = 0;
        virtual const State& getState() const = 0;
        virtual Eigen::MatrixXd getCovariance() const = 0;

};  