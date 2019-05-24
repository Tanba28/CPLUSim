#ifndef __ENVIROMENT_HPP__
#define __ENVIROMENT_HPP__

#include <Eigen/Core>

class Enviroment{
    public:
        double R = 287.1;
        double g0 = 9.80665;
        double temp0 = 273.15 + 15;
        double press0 = 101315.0;
        double rho0 = press0/(R*temp0);

        double velWind0 = 3.0;
        double dirWind0 = 0.0;
        double baseHeight = 2.0;
        double windPower = 7.4;

        double azimuth0 = 0;
        double elevation0 = -89.0;

        double railLength = 5.0;

        Eigen::Vector3d windLaw(double height);
        void atmosphere(double height, double &temp, double &press, double &rho);
};
#endif