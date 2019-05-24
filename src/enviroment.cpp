#define _USE_MATH_DEFINES

#include <iostream>
#include <Eigen/Core>
#include <cmath>

#include "enviroment.hpp"
#include "coordinate.hpp"

Eigen::Vector3d Enviroment::windLaw(double height){
    Eigen::Vector3d velWind;
    double velWindAbs;
    if(height > 0.0){
        velWindAbs = velWind0 * pow((height / baseHeight),(1/windPower));
    }
    else{
        velWindAbs = 0;
    }

    velWind(0) = -velWindAbs * cos(deg2rad(dirWind0));
    velWind(1) = -velWindAbs * sin(deg2rad(dirWind0));
    velWind(2) = 0.0;

    return velWind;
}
void Enviroment::atmosphere(double height, double &temp, double &press, double &rho){
    double tempd = 0.0065;
    temp = temp0 - tempd*height;
    press = press0 * pow((temp/temp0),(g0/tempd/R));
    rho = press/(R*temp);
}
