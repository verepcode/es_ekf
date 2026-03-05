//include/es_ekf/sensors/GNSSSensor.hpp
#pragma once
#include "es_ekf/sensors/SensorBase.hpp"

namespace es_ekf{
class GNSSSensor : public SensorBase{
    public: 
        GNSSSensor(double variance = 1.0)
            : variance_(variance)
            {}

        static constexpr int MEAS_DIM = 3;

        //Noise covariance (R matrisi)
        Eigen::MatrixXd getNoiseCovariance() const override {
            return variance_ * Eigen::Matrix3d::Identity();
        }
        //Variance setter/getter
        void setVariance(double variance) {variance_ = variance;}
        double getVariance() const {return variance_;}

        
    private:
        double variance_ = 1.0;
};
}