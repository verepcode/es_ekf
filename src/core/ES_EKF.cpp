#include "filters/ESEKF.hpp"

State ESEKF::correctState(const State& predicted, const Eigen::Matrix<double,9,1>& delta){
    State s = predicted;
    s.applyErrorState(delta);
    return s;
}