#ifndef __ROCKET_HPP__
#define __ROCKET_HPP__

#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <string>
#include <Eigen/Core>

class RocketParameter{
    public:        
        double thrustTime = 4;
        double thrust = 300;

        double length = 1.3;
        double diameter = 0.089;
        double area = 0.25*M_PI*diameter*diameter;

        double mass = 5.0;

        double inertiaPitch = 0.6;
        double inertiaRoll = 0.006;

        double lcg = 0.8;
        double lcp = 0.95;

        double cd = 0.5;
        double cna = 8.0;
        double clp = -0.07;
        double cmq = -2.5;
};

#endif