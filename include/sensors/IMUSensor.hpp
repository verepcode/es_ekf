//include/sensors/IMUSensor.hpp
#pragma once

#include "sensors/SensorBase.hpp"


class IMUSensor : public SensorBase{

    public:
        IMUSensor(double variance = 1.0)
            : variance_(variance)
            {}

            Eigen::MatrixXd getNoiseCovariance() const{
            
                return variance_ * Eigen::Matrix3d::Identity();
            }

            //Variance getter/setter
            double getVariance() const { return variance_; }
            void setVariance(double variance) { variance_ = variance; }
    private:
            double variance_ = 1.0;
}