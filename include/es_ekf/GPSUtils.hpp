//include/es_ekf/GPSUtils.hpp

#pragma once
#include <optional>
#include <Eigen/Dense>
#include "es_ekf/MathUtils.hpp"
#include <iomanip>

namespace es_ekf{

class GPSUtils
/*
    Contains the algorithms to convert a gps signal (longitude, latitude, height)
    to a local cartesian ENU system and vice versa

    Use setENUorigin(lat, lon, height) to set the local ENU coordinate system origin
    Use geo2enu(lat, lon, height) to get the position in the local ENU system
    Use enu2geo(x_enu, y_enu, z_enu) to get the latitude, longitude and height */
{
public:
    GPSUtils()
        // Geodetic System WGS 84 axes
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
        origin_ecef_ = geo2Ecef(lat_zero, lon_zero, height_zero);

        double phi    = degToRad(lat_zero);
        double lambda = degToRad(lon_zero);

        double c_phi = std::cos(phi);
        double c_lambda = std::cos(lambda);
        double s_phi = std::sin(phi);
        double s_lambda = std::sin(lambda);

        R_ <<       -s_lambda,          c_lambda,     0.0,
            -s_phi * c_lambda, -s_phi * s_lambda,   c_phi,
             c_phi * c_lambda,  c_phi * s_lambda,   s_phi;

    }

    const std::string printEcefOrigin() const{
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        oss << "╔════════════════════════════════╗\n"
            << "║         ECEF Origin            ║\n"
            << "╠════════════════════════════════╣\n"
            << "║  X: " << std::setw(14) << origin_ecef_.x() << " m  ║\n"
            << "║  Y: " << std::setw(14) << origin_ecef_.y() << " m  ║\n"
            << "║  Z: " << std::setw(14) << origin_ecef_.z() << " m  ║\n"
            << "╚════════════════════════════════╝";
        return oss.str();
    }
    const std::string pringRotationMAtrix() const{
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        oss << "╔════════════════════════════════╗\n"
            << "║   ECEF → ENU Rotation Matrix   ║\n"
            << "╠════════════════════════════════╣\n"
            << R_ << "\n"
            << "╚════════════════════════════════╝";
        return oss.str();
    }
    std::string printGPSCoord(const double& lat, const double& lon, const double& height) const{
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        oss << "╔════════════════════════════════╗\n"
            << "║        GPS Coordinates         ║\n"
            << "╠════════════════════════════════╣\n"
            << "║  Lat:    " << std::setw(12) << lat << " °  ║\n"
            << "║  Lon:    " << std::setw(12) << lon << " °  ║\n"
            << "║  Height: " << std::setw(12) << height << " m  ║\n"
            << "╚════════════════════════════════╝";
        return oss.str();
    }

    const Eigen::Vector3d getEcefOrigin() const
    {
        return origin_ecef_;
    }

    Eigen::Vector3d geo2Ecef(const double &lat, const double &lon, const double& height) const
    {
        double phi    = degToRad(lat);
        double lambda = degToRad(lon);

        double c_phi    = std::cos(phi);
        double c_lambda = std::cos(lambda);
        double s_phi    = std::sin(phi);
        double s_lambda = std::sin(lambda);

        double N = a_ / std::sqrt(1.0 - e2_ * s_phi * s_phi);

        double x = (N + height) * c_phi * c_lambda;
        double y = (N + height) * c_phi * s_lambda;
        double z = ((b2_ / a2_) * N + height) * s_phi;

        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Vector3d ecefToEnu(const double& x, const double& y, const double& z) const
    {
        Eigen::Vector3d ecef(x, y, z);
        
        return R_ * (ecef - origin_ecef_);
    }

    Eigen::Vector3d geoToEnu(const double& lat, const double& lon, const double& height) const
    {
        auto ecef = geo2Ecef(lat, lon, height);

        return ecefToEnu(ecef[0], ecef[1], ecef[2]);
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


    Eigen::Matrix3d R_;
    Eigen::Vector3d origin_ecef_;
};
}