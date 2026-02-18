//include/es_ekf/filters/ESEKF.hpp
#pragma once

#include "es_ekf/filters/FilterBase.hpp"
#include "es_ekf/models/MotionModel.hpp"
#include "es_ekf/sensors/SensorBase.hpp"
#include <memory>
#include <vector>

namespace es_ekf{

class ESEKF : public FilterBase{

    public:
        ESEKF();
        //Set motion model
        void setMotionModel(std::shared_ptr<MotionModel> model){
            motion_model_ = model;
        }
        //Add sensor
        void addSensor(std::shared_ptr<SensorBase> sensor) {
            sensors_.push_back(sensor);
        }
        //Implement filterbase
        PredictionResult predict(const ImuData& imu, double dt) override;
        UpdateResult update(const MeasurementData& meas) override;

        const State& getState() const override {return state_hat_;}
        const Eigen::MatrixXd& getCovariance() const override { return p_cov_;}
    
    private:
        State state_check_;
        State state_hat_;
        Eigen::MatrixXd p_cov_;
        std::shared_ptr<MotionModel> motion_model_;
        std::vector<std::shared_ptr<SensorBase>> sensors_;    
};
}//namespace es_ekf