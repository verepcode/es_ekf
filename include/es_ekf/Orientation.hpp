// include/es_ekf/Orientation.hpp

/*
    Allow initialization with explicit quaterion wxyz, axis-angle, or Euler XYZ (RPY) angles.

    :param w: w (real) of quaternion.
    :param x: x (i) of quaternion.
    :param y: y (j) of quaternion.
    :param z: z (k) of quaternion.
    :param axis_angle: Set of three values from axis-angle representation, as list or [3,] or [3,1] np.ndarray.
                        See C2M5L2 for details.
    :param euler: Set of three XYZ Euler angles.*/

    #pragma once

#include <Eigen/Dense>
#include <cmath>

namespace es_ekf{

class Orientation {
public:
    //default constructor - Orientation() çağrıldığında
    Orientation() 
        : q_(Eigen::Quaterniond::Identity()) 
    {}
    const Eigen::Quaterniond& quaternion() const { return q_}
    
    static Orientation fromEuler(double roll, double pitch, double yaw){
        double cy = std::cos(yaw / 2);
        double sy = std::sin(yaw / 2);
        double cr = std::cos(roll / 2);
        double sr = std::sin(roll / 2);
        double cp = std::cos(pitch / 2);
        double sp = std::sin(pitch / 2);

        // Fixed frame
        double w = cr * cp * cy + sr * sp * sy;
        double x = sr * cp * cy - cr * sp * sy;
        double y = cr * sp * cy + sr * cp * sy;
        double z = cr * cp * sy - sr * sp * cy;

        // Rotating frame
        // double w = cr * cp * cy - sr * sp * sy;
        // double x = cr * sp * sy + sr * cp * cy;
        // double y = cr * sp * cy - sr * cp * sy;  
        // double z = cr * cp * sy + sr * sp * cy;

        return Orientation(Eigen::Quaterniond(w, x, y, z));
    }

    static Orientation fromAxisAngle(const Eigen::Vector3d& axis_angle){
        double norm = axis_angle.norm();
        
        if (norm < 1e-10){
            return Orientation(); // Identity quaternion;
        }
        double w = cos(norm / 2.0);
        Eigen::Vector3d imag = axis_angle / norm * sin(norm / 2.0);
        double x = imag[0];
        double y = imag[1];
        double z = imag[2];
            
        return Orientation(Eigen::Quaterniond(w, x, y, z));
    }
    
    static Orientation fromQuaternion(double w, double x, double y, double z){
        return Orientation(Eigen::Quaterniond(w, x, y, z).normalized());
    }

    Eigen::Matrix4d quatMultRightMatrix() const {
        Eigen::Vector3d v(q_.x(), q_.y(), q_.z());
        Eigen::Matrix4d sum_term = Eigen::Matrix4d::Zero();

        sum_term(0,1) = -v.x();
        sum_term(0,2) = -v.y();
        sum_term(0,3) = -v.z();
        sum_term(1,0) = v.x();
        sum_term(2,0) = v.y();
        sum_term(3,0) = v.z();

        sum_term.block<3, 3>(1, 1) = -skewSymmetric(v);
        Eigen::Matrix4d sigma = q_.w() * Eigen::Matrix4d::Identity() + sum_term;
        return sigma;
    }
    
    Eigen::Matrix4d quatMultLeftMatrix() const{
        Eigen::Vector3d v(q_.x(), q_.y(), q_.z());
        Eigen::Matrix4d sum_term = Eigen::Matrix4d::Zero();

        sum_term(0, 1) = -v.x();
        sum_term(0, 2) = -v.y();
        sum_term(0, 3) = -v.z();
        sum_term(1, 0) = v.x();
        sum_term(2, 0) = v.y();
        sum_term(3, 0) = v.z();

        sum_term.block<3, 3>(1, 1) = skewSymmetric(v);
        Eigen::Matrix4d sigma = q_.w() * Eigen::Matrix4d::Identity() + sum_term;
        return sigma;
    }

    Orientation quatMultRight(const Orientation&  other) const {
        Eigen::Matrix4d sigma = quatMultRightMatrix();
        Eigen::Vector4d q_vec(other.q_.w(), other.q_.x(), other.q_.y(), other.q_.z());
        Eigen::Vector4d result = sigma * q_vec;

        return Orientation(Eigen::Quaterniond(result(0), result(1), result(2), result(3)));
    }

    Orientation quatMultLeft(const Orientation &other) const
    {
        Eigen::Matrix4d sigma = quatMultLeftMatrix();
        Eigen::Vector4d q_vec(other.q_.w(), other.q_.x(), other.q_.y(), other.q_.z());
        Eigen::Vector4d result = sigma * q_vec;

        return Orientation(Eigen::Quaterniond(result(0), result(1), result(2), result(3)));
    }

    Eigen::Vector4d quatMultRightVec(const Orientation &other) const {
        Eigen::Matrix4d sigma = quatMultRightMatrix();
        Eigen::Vector4d q_vec(other.q_.w(), other.q_.x(), other.q_.y(), other.q_.z());

        return  sigma * q_vec;
    }
    
    Eigen::Vector4d quatMultLeftVec(const Orientation &other) const
    {
        Eigen::Matrix4d sigma = quatMultLeftMatrix();
        Eigen::Vector4d q_vec(other.q_.w(), other.q_.x(), other.q_.y(), other.q_.z());

        return sigma * q_vec;
    }

    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &vector) const{
        Eigen::Matrix3d result_matrix = Eigen::Matrix3d::Zero();
        result_matrix << 0, -vector.z(),  vector.y(),
                vector.z(),           0, -vector.x(),
               -vector.y(),  vector.x(),           0;
        
        return result_matrix;
    }

    Eigen::Vector3d toEuler() const {
        double roll = std::atan2(2 * (q_.w() * q_.x() + q_.y() * q_.z()),
                            1 - 2 * (q_.x() * q_.x() + q_.y() * q_.y()));
        double pitch = std::asin(2 * (q_.w() * q_.y() - q_.z() * q_.x()));
        double yaw = std::atan2(2 * (q_.w() * q_.z() + q_.x() * q_.y()), 
                            1 - 2 * (q_.y() * q_.y() + q_.z() * q_.z()));
        
        return Eigen::Vector3d(roll, pitch, yaw);
    }

    Eigen::Vector3d toAxisAngle() const {
        
        double theta = 2 * std::acos(q_.w());

        if(theta < 1e-10){
            return Eigen::Vector3d::Zero(); // Zero for infinitesimal angles
        }

        Eigen::Vector3d axis(q_.x(), q_.y(), q_.z());
        Eigen::Vector3d axis_normalized = axis / sin(theta / 2);

        return axis_normalized * theta;
    }

    Eigen::Matrix3d toMatrix() const {
        Eigen::Vector3d v(q_.x(), q_.y(), q_.z());
        double w = q_.w();
        double vTv = v.dot(v);
        Eigen::Matrix3d vvT = v * v.transpose();

        return (w * w - vTv) * Eigen::Matrix3d::Identity()
                + 2.0 * vvT
                + 2.0 * w * skewSymmetric(v);
    }
    
    Orientation& normalize(){
        Eigen::Vector4d vector(q_.w(), q_.x(), q_.y(), q_.z());
        double norm = vector.norm();
        q_ = Eigen::Quaterniond((q_.w()) / norm, (q_.x()) / norm, (q_.y()) / norm, (q_.z()) / norm);
        return *this;
    }
    
    private: 
    Eigen::Quaterniond q_;

    // Private constructor - sadece factory metodlar kullanır
    explicit Orientation(const Eigen::Quaterniond& q) : q_(q) {}
};
} //namespace es_ekf