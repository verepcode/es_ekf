#include  "es_ekf/Types.hpp"
#include "es_ekf/State.hpp"
#include "es_ekf/Orientation.hpp"
#include <Eigen/Dense>

using namespace es_ekf;

class StateEstimator 
{

public:
    StateEstimator()
    {
        h_jac_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        l_jac_.block<6, 6>(3, 0) = Eigen::Matrix<double, 6, 6>::Identity();
    }

    PredictionResult predict(const ImuData& imu, const double& delta_t){

        Eigen::Vector3d f = imu.linear_accel;
        Eigen::Vector3d w = imu.ang_vel;
        auto rot_mat= state_hat_.orientation.toMatrix();
        Eigen::Vector3d acc = rot_mat * f + gravity_mat_;
        state_check_.position = state_hat_.position + state_hat_.velocity * delta_t + acc * (delta_t*delta_t/2);
        state_check_.velocity = state_hat_.velocity + acc * delta_t;
        state_check_.orientation = Orientation::fromAxisAngle(w * delta_t)
                                    .quatMultRight(state_hat_.orientation);
        
        f_jac_.block<3, 3>(0);
        
    }
    UpdateResult update(const GNSSData& gnss){

    }

private :

    double gravity_ = -9.81;
    Eigen::Vector3d gravity_mat_ = Eigen::Vector3d(0, 0, gravity_);
    State state_check_;
    State state_hat_;
    Eigen::Matrix<double, 3, 9> h_jac_ = Eigen::Matrix<double, 3, 9>::Zero();
    Eigen::Matrix<double, 9, 6> l_jac_ = Eigen::Matrix<double, 9, 6>::Zero();
    Eigen::Matrix<double, 9, 9> f_jac_ = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix3d m_jac_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d r_est_euler_;    //orientation estimates as euler angles
    Eigen::Vector3d r_est_euler_std; //standart deviation od orientation estimates
    Eigen::Vector3d speed_est_;
    double var_imu_f_ = 3;
    double var_imu_w_ = 15;
    double var_gnss_ = 8;
};