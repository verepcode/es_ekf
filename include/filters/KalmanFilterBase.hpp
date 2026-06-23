//include/filters/KalmanFilterBase.hpp
#pragma once

#include "filters/FilterBase.hpp"
#include "models/MotionModel.hpp"
#include "sensors/SensorBase.hpp"
#include <memory>
#include <vector>

class KalmanFilterBase : public FilterBase {
    public:
        KalmanFilterBase();
        State predict(const ImuData& imu, double dt) override;
        State update(const MeasurementData& meas, State& state_check) override;

        const State& getState() const override { return state_; }
        Eigen::MatrixXd getCovariance() const override { return state_.covariance; }
        //Set motion model
        void setMotionModel(std::shared_ptr<MotionModel> model){
            motion_model_ = model;
        }
        //Add sensor
        void addSensor(std::shared_ptr<SensorBase> sensor) {
            sensors_.push_back(sensor);
        }
    protected:
        virtual State correctState(const State& predicted, const Eigen::Matrix<double,9,1>& delta)=0;
        std::shared_ptr<MotionModel> motion_model_;
        std::vector<std::shared_ptr<SensorBase>> sensors_;
        double gravity_ = -9.81;
        Eigen::Vector3d gravity_mat_ = Eigen::Vector3d(0, 0, gravity_);
        State state_;
        Eigen::Matrix<double, 3, 9> h_jac_ = Eigen::Matrix<double, 3, 9>::Zero();
        Eigen::Matrix<double, 9, 6> l_jac_ = Eigen::Matrix<double, 9, 6>::Zero();
        Eigen::Matrix<double, 9, 9> f_jac_ = Eigen::Matrix<double, 9, 9>::Zero();

        double var_imu_f_ = 3;
        double var_imu_w_ = 15;
        double var_gnss_ = 8;
};      