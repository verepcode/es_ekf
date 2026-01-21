//include/es_ekf/Utils.hpp
/*Explanations*/
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace es_ekf{
    using std::sqrt;

    inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d v) const {

        Eigen::Matrix3d result_matrix = Eigen::Matrix3d::Zero();
        result_matrix << 0, -v[2], v[1],
                      v[2],     0,-v[0],
                     -v[1],  v[0],    0;  
        return result_matrix;
    }
    inline double angleNormalize(double angle) {
        return std::remainder(angle, 2 * M_PI);  // returns angle between (-pi,pi]
    }
    inline Eigen::Matrix3d rpyJacobianAxisAngle(const Eigen::Vector3d& a){
        //Jacobian of RPY Euler angles with respect to axis-angle vector.
        double na = a.norm();
        double t = a.norm();
        double na3 = na * na * na;
        Eigen::Vector3d u = a / t;

        //Jr 3x4 bir matris
        Eigen::Matrix<double, 3, 4> Jr = Eigen::Matrix<double, 3, 4>::Zero();
        double denom0 = t*t * u[0]*u[0] + 1;
        double denom1 = sqrt(std::max(0.01, 1 - t*t * u[1]*u[1]));
        double denom2 = t*t * u[2]*u[2] + 1;
        Jr(0,0) = t/denom0;
        Jr(0,3) = u[0]/denom0;
        Jr(1,1) = t/denom1;
        Jr(1,3) = u[1]/denom1;
        Jr(2,2) = t/denom2;
        Jr(2,3) = u[2]/denom2;

        //Ja 4x3 bir matris
        Eigen::Matrix<double, 4, 3> Ja = Eigen::Matrix<double, 4, 3>::Zero();

        Ja << (a[1]*a[1] + a[2]*a[2])/na3,               -a[0]*a[1]/na3,              -a[0]*a[2]/na3,
                        -a[0]*a[1]/na3,  (a[0]*a[0] + a[2]*a[2])/na3,              -a[1]*a[2]/na3,
                        -a[0]*a[2]/na3,               -a[1]*a[2]/na3, (a[0]*a[0] + a[1]*a[1])/na3,
                                a[0]/na,                      a[1]/na,                     a[2]/na;
        return Jr * Ja;
    }
    inline double degToRad(const double& degree){
        return degree / 180 * M_PI;
    }
    inline double radToDeg(const double& radian){
        return radian / M_PI * 180;
    }
}