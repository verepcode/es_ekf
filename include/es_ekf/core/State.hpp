// include/es_ekf/State.hpp
#pragma once
 
#include <Eigen/Dense>
#include "es_ekf/core/Orientation.hpp"

namespace es_ekf {

    struct State {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        double speed = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);  
        Orientation orient_in_quat;
        Eigen::Vector3d orient_in_euler = orient_in_quat.toEuler();
        Eigen::Matrix<double, 9, 9> covariance = Eigen::Matrix<double, 9, 9>::Zero();
        // Toplam error-state boyutu: 3 + 3 + 3 = 9 };
        // orientation error 3, quaternion değil

        static constexpr int ERROR_STATE_DIM = 9;
        //Default constructor
        State()
        {}

        //Error-state'i nominal state'e uygula
        void applyErrorState(const Eigen::VectorXd& delta) {
            position += delta.segment<3>(0);
            velocity += delta.segment<3>(3);
            orient_in_quat.applyErrorState(delta.segment<3>(6));
        }
    };

} //namespace es_ekf