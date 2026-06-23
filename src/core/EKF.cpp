//src/core/EKF.cpp

#include "filters/EKF.hpp"
#include "core/Orientation.hpp"

State EKF::correctState(const State& predicted, const Eigen::Matrix<double,9,1>& delta){
    State s = predicted;
    s.position += delta.segment<3>(0);
    s.velocity += delta.segment<3>(3);
    Eigen::Vector3d dEuler = delta.segment<3>(6);
    s.orient_in_quat = Orientation::fromEuler(s.orient_in_quat.toEuler() + dEuler);

    return s;
}
