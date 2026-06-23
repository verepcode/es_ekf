//include/filters/EKF.hpp

#pragma once

#include "filters/KalmanFilterBase.hpp"

class EKF : public KalmanFilterBase {
    protected:
        State correctState(const State& predicted, const Eigen::Matrix<double,9,1>& delta) override;

};      