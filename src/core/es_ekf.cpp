#include  "es_ekf/Types.hpp"
#include <Eigen/Dense>

class StateEstimator 
{

public:
    StateEstimator()
    : gravity_(-9.81)
    , l_jacobian_(Eigen::Matrix<double, 9, 6>::Zero())
    , m_jacobian_(Eigen::Matrix3d::Identity())
    , h_jacobian_(Eigen::Matrix<double, 3, 9>::Zero())
    , p_est_(Eigen::MatrixXd::Zero())



{}

private:

    double gravity_ ;
    Eigen::MatrixXd l_jacobian_;
    Eigen::MatrixXd m_jacobian_;
    Eigen::MatrixXd h_jacobian_;
    Eigen::MatrixXd p_est_, v_est_;




};