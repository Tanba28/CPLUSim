#ifndef __ROCKET_HPP__
#define __ROCKET_HPP__

#define _USE_MATH_DEFINES

#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <Eigen/Core>

#include "picojson.h"

class Rocket{
    public:
        Rocket(picojson::object json);
        
        double mass0;

        // System
        std::string thrustFileName;
        std::string rocketName;
        bool modelRocket;

        // Mechanics
        double length;
        double diameter;
        double area;
        double structureMass;
        double structureCG;

        double inertiaPitch;
        double inertiaRoll;

        // AeroDynamics
        double lcp;
        double clp;
        double cmq;
        double cd;
        double cna;

        //TODO:Recovery
        
        double deployTimePara1;
        double decentVelPara1;
        
        bool parachute2;
        double deployAltitudePara2;
        double decentVelPara2;
        

        //TODO:HybridEngine
        
        double isp;
        double oxidizerTankLength;
        double oxidizerTankCG;
        double oxidizerMass;
        double oxidizerMassDot;
        double fuelLength;
        double fuelCG;
        double fuelMassBefore;
        double fuelMassAfter;
        double fuelMassDot;
        

        //TODO:ModelRocketEngine
        
        double motorLength;
        double motorDiameter;
        double motorCG;
        double motorMassBefore;
        double motorMassAfter;
        
        double thrustTime;
        //double thrust = 300;
        Eigen::MatrixXd thrust;
};

#endif