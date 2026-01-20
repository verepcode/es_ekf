//include/es_ekf/GPSUtils.hpp

#pragma once
#include <optional>
#include <Eigen/Dense>

class GPSUtils {


private:
    const double a_;
    const double b_;
    const double a2_;
    const double b2_;
    const double e2_;
    const double e_;

    std::optional<double> lat_zero_;
    std::optional<double> lon_zero_;
    std::optional<double> hgt_zero_;
    std::optional<double> x_zero_;
    std::optional<double> y_zero_;
    std::optional<double> z_zero_;

    Eigen::Matrix3d R_;
}