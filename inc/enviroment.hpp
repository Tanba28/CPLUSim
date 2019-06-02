#ifndef __ENVIROMENT_HPP__
#define __ENVIROMENT_HPP__

#define _USE_MATH_DEFINES

#include <iostream>
#include <Eigen/Core>
#include <cmath>
#include <Eigen/Core>

#include "picojson.h"
#include "coordinate.hpp"

class Enviroment{
    public:
        Enviroment(picojson::object json);
        double R = 287.1;
        double g0 = 9.80665;
        double temp0 = 273.15 + 15;
        double press0 = 101315.0;
        double rho0 = press0/(R*temp0);

        double velWind0;
        double dirWind0;
        double baseHeight;
        double windPower;

        double azimuth0;
        double elevation0;

        double railLength;

        Eigen::Vector3d windLaw(double height);
        void atmosphere(double height, double &temp, double &press, double &rho);
};
#endif