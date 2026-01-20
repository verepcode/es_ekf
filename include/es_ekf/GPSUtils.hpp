//include/es_ekf/GPSUtils.hpp

#pragma once
#include <optional>
#include <Eigen/Dense>

namespace es_ekf{

class GPSUtils
{
public:
    GPSUtils()
        : a_(6378137.0)
        , b_(6356752.314245)
        , a2_(a_ * a_)
        , b2_(b_ * b_)
        , e2_(1.0 - (b2_ / a2_))
        , e_(e2_ / (1.0 - e2_))
        , R_(Eigen::Matrix3d::Identity())

    {}
    void setENUOrigin(const double& lat_zero, const double& lon_zero, const double& height_zero)
    {
        auto origin = geo2Ecef(lat_zero, lon_zero, height_zero);
        auto x_zero = origin[0];
        auto y_zero = origin[1];
        auto z_zero = origin[2];
        // auto o_zero = origin;

    }
    Eigen::Vector3d geo2Ecef(const double &lat, const double &lon, const double& height){

    }

    private :
    // WGS84 constants
    const double a_;
    const double b_;
    const double a2_;
    const double b2_;
    const double e2_;
    const double e_;

    // Local ENU origin
    std::optional<double> lat_zero_;
    std::optional<double> lon_zero_;
    std::optional<double> hgt_zero_;
    std::optional<double> x_zero_;
    std::optional<double> y_zero_;
    std::optional<double> z_zero_;

    Eigen::Matrix3d R_;
};
}