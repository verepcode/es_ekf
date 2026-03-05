//include/es_ekf/filters/EKF.hpp

#pragma once

#include "es_ekf/filters/FilterBase.hpp"

class EKF : public FilterBase {
    public:
        EKF();

        // Set motion model
        void setMotionModel(std::shared_ptr<MotionModel> model)
        {
            motion_model_ = model;
        }
        // Add sensor
        void addSensor(std::shared_ptr<SensorBase> sensor)
        {
            sensors_.push_back(sensor);
        }

        PredictionResult predict(const ImuData& imu, double dt) override {

        }
        UpdateResult update(const MeasurementData& meas) override {

        }

        const State& getState() const { return state_hat_;}
        const Eigen::MatrixXd& getCovariance() const { return p_cov_;}
    private:

        State state_check_;
        State state_hat_;
        Eigen::MatrixXd p_cov_;
        std::shared_ptr<MotionModel> motion_model_;
        std::vector<std::shared_ptr<SensorBase>> sensors_;
};      