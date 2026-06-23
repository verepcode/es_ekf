//include/filters/ESEKF.hpp
#pragma once

#include "filters/ESEKF.hpp"


class ESEKF : public KalmanFilterBase {
    protected:
        State correctState(const State& predicted, const Eigen::Matrix<double,9,1>& delta) override;

};      