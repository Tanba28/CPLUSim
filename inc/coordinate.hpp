#ifndef __COORDINATE_HPP__
#define __COORDINATE_HPP__
#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>

#include <Eigen/Core>

namespace Coordinate{
    Eigen::Matrix3d convertQuatToDcm(Eigen::Vector4d quat);
    Eigen::Matrix3d convertEulerToDcm(Eigen::Vector3d euler);
    Eigen::Vector3d convertQuatToEuler(Eigen::Vector4d quat);
    Eigen::Vector4d convertDcmToQuat(Eigen::Matrix3d dcm);
};

inline double deg2rad(double deg){
    return deg * M_PI /180.0;
}

inline double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}

#endif