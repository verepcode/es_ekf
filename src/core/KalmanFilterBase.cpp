//src/core/KalmanFilterBase.cpp

#include "filters/KalmanFilterBase.hpp"
#include "core/Orientation.hpp"
#include "MathUtils.hpp"


KalmanFilterBase::KalmanFilterBase(){
        h_jac_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        l_jac_.block<6, 6>(3, 0) = Eigen::Matrix<double, 6, 6>::Identity();
    }

State KalmanFilterBase::predict(const ImuData& imu, double dt){

    Eigen::Vector3d f = imu.linear_accel;
    Eigen::Vector3d w = imu.ang_vel;
    auto rot_mat = state_.orient_in_quat.toMatrix();
    Eigen::Vector3d acc = rot_mat * f + gravity_mat_;
    State state_check;
    state_check.position = state_.position + state_.velocity * dt + acc * (dt*dt/2);
    state_check.velocity = state_.velocity + acc*dt;
    state_check.orient_in_quat = Orientation::fromAxisAngle(w * dt)
                                        .quatMultRight(state_.orient_in_quat);
    // Linearize the motion model and compute Jacobians
    // motion model state jacobian
    f_jac_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    f_jac_.block<3, 3>(3, 6) = -rot_mat * skewSymmetric(f) * dt;
    // Propagate uncertainty
    Eigen::Matrix<double, 6, 1> v;
    v.head<3>().setConstant(var_imu_f_);
    v.tail<3>().setConstant(var_imu_w_);

    // Now, v = [var_imu_f_, var_imu_f_, var_imu_f_, var_imu_w_, var_imu_w_, var_imu_w_]
    // Then, make v a diagonal matris using an Eigen method asDiagonal
    Eigen::Matrix<double, 6, 6> q_km = dt * dt * v.asDiagonal();
    state_check.covariance = f_jac_ * (state_.covariance * f_jac_.transpose()) +
                                l_jac_ * (q_km * l_jac_.transpose());
    
    state_ = state_check;
    return state_check;
}
State KalmanFilterBase::update(const MeasurementData &meas, State& state_check){

    Eigen::Matrix3d R = meas.covariance; 
    Eigen::Matrix<double, 9, 9> cov_check = state_check.covariance;
    // Compute Kalman gain
    Eigen::Matrix<double, 9, 3> K_k = cov_check * h_jac_.transpose() * (h_jac_ * cov_check * h_jac_.transpose() + R).inverse();
    Eigen::Matrix<double, 9, 1> delta = K_k * (meas.value - state_check.position);
    
    State state_hat = correctState(state_check, delta);
    state_hat.covariance = (Eigen::Matrix<double,9,9>::Identity() - K_k * h_jac_) * cov_check;


    state_ = state_hat;
    return state_hat;
}

