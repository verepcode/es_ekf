// include/es_ekf/State.hpp
#pragma once
 
#include <Eigen/Dense>
#include "es_ekf/Orientation.hpp"

namespace es_ekf {

struct State {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Orientation orientation;
    // Toplam error-state boyutu: 3 + 3 + 3 = 9 };
    // orientation error 3, quaternion değil

    static constexpr int ERROR_STATE_DIM = 9;
    //Default constructor
    State()
        : position(Eigen::Vector3d::Zero()),
          velocity(Eigen::Vector3d::Zero()),
          orientation() 
    {}

    //Error-state'i nominal state'e uygula
    void applyErrorState(const Eigen::VectorXd& delta) {
        position += delta.segment<3>(0);
        velocity += delta.segment<3>(3);
        orientation.applyErrorState(delta.segment<3>(6));
    }



};

} //namespace es_ekf