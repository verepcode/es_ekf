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

