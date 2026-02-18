// include/es_ekf/core/types.hpp
#pragma once

#include <Eigen/Dense>
#include "es_ekf/core/State.hpp"

namespace es_ekf{

    struct ImuData
    {
        Eigen::Vector3d linear_accel;
        Eigen::Vector3d ang_vel;
        double timestamp;
    };

    // Abstract/base for measurement data
    struct MeasurementData
    {
        Eigen::VectorXd value;
        Eigen::MatrixXd covariance;
        double timestamp = 0.0;

        virtual ~MeasurementData() = default;
    };


    struct GNSSData : public MeasurementData{
        GNSSData()
        {
            value = Eigen::Vector3d::Zero();
            covariance = Eigen::Matrix3d::Identity();
        }
        GNSSData(const Eigen::Vector3d& position, double variance = 1.0){
            value = position;
            covariance = variance * Eigen::Matrix3d::Identity();
        }
        //Kolay erişim
        Eigen::Vector3d position() const { return value; }
    };

    struct CameraData : public MeasurementData{
        std::vector<Eigen::Vector2d> feature_points;
        std::vector<int> feature_ids;
        CameraData()
        {
            covariance = Eigen::Matrix2d::Identity(); //Pixel noise
        }
    };
    

    struct PredictionResult
    {
        State state_check;
    };

    struct UpdateResult
    {
        Eigen::VectorXd state;
        Eigen::VectorXd covariance;
        Eigen::VectorXd innovation;
        Eigen::VectorXd kalman_gain;
    };
}
