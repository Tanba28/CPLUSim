#define _USE_MATH_DEFINES

#include <iostream>
#include <Eigen/Core>
#include <boost/numeric/odeint.hpp>
#include <cmath>

#include "dynamics.hpp"
#include "coordinate.hpp"

Dynamics::Dynamics(Parameter *inputParameter){
    parameter = inputParameter;
}

void Dynamics::operator()(const state &x, state &dx, const double t){
    using namespace Eigen;
    using namespace std;

    Vector3d posEnu;
    Vector3d velEnu;
    Vector3d omega;
    Vector4d quat;
    double mass;

    Matrix3d dcm;
    Vector3d accEnu;
    Vector3d omegaDot;
    Matrix4d omegaDotMatrix;
    Vector4d quatDot;
    double massDot;

    double temp,press,rho;
    Vector3d g;

    Vector3d velWind;
    Vector3d velAir;
    double velAirNorm;

    bool launcheCrear;

    double alpha,beta;
    double dragForce,sideForce,normalForce;
    double thrustForce;
    Vector3d totalForce;

    Vector3d aeroMoment;
    Vector3d aeroDampingMoment;
    Vector3d jetDampingMoment;
    Vector3d totalMoment;

    posEnu << x(0), x(1), x(2);
    velEnu << x(3), x(4), x(5);
    omega << x(6), x(7) ,x(8);
    quat << x(9), x(10), x(11), x(12);
    mass = x(13);

    if(posEnu(2) < 0){
        dx << MatrixXd::Zero(14,1);
        return;
    }
    quat.normalized();

    dcm = Coordinate::convertQuatToDcm(quat);

    launcheCrear = (posEnu.norm() > parameter->enviroment.railLength) && (posEnu(2) > 0);

    parameter->enviroment.atmosphere(posEnu(2),temp,press,rho);
    g << 0,0,-parameter->enviroment.g0;

    velWind = parameter->enviroment.windLaw(posEnu(2));
    velAir = dcm*(velEnu-velWind);
    velAirNorm = velAir.norm();

    if((velAirNorm == 0) || (launcheCrear == false)){
        alpha = 0.0;
        beta = 0.0;
    }
    else{
        alpha = atan2(velAir(2),velAir(0));
        beta = asin(-velAir(1)/velAirNorm);
    }

    if(t < parameter->rocket.thrustTime){
        //thrustForce = parameter->rocket.thrust;
        //thrustForce = 300;
        thrustForce = interp1d(parameter->rocket.thrust,t);
        massDot = -(parameter->rocket.oxidizerMassDot + parameter->rocket.fuelMassDot)/100;
    }
    else{
        thrustForce = 0;
        massDot = 0;
    }

    dragForce = 0.5 * rho * (velAirNorm*velAirNorm) * parameter->rocket.area * parameter->rocket.cd;
    sideForce = 0.5 * rho * (velAirNorm*velAirNorm) * parameter->rocket.area * parameter->rocket.cna * beta;
    normalForce = 0.5 * rho * (velAirNorm*velAirNorm) * parameter->rocket.area * parameter->rocket.cna * alpha;

    totalForce << thrustForce - dragForce*cos(beta)*cos(alpha) ,sideForce, -normalForce;


    accEnu = (dcm.transpose()*(totalForce/parameter->rocket.mass0)) + g;


    if((dcm*accEnu)(0) < 0 && launcheCrear == false){
        accEnu << 0,0,0;
    }

    aeroMoment << 
        0, 
        -normalForce*(parameter->rocket.lcp-parameter->rocket.structureCG),
        -sideForce*(parameter->rocket.lcp-parameter->rocket.structureCG);

    aeroDampingMoment <<
        0.25 * rho * velAirNorm * parameter->rocket.area * parameter->rocket.diameter*parameter->rocket.diameter * parameter->rocket.clp * omega(0),
        0.25 * rho * velAirNorm * parameter->rocket.area * parameter->rocket.length*parameter->rocket.length * parameter->rocket.cmq * omega(1),
        0.25 * rho * velAirNorm * parameter->rocket.area * parameter->rocket.length*parameter->rocket.length * parameter->rocket.cmq * omega(2);
    
    // ToDo:jetDampingMoment

    totalMoment = aeroMoment + aeroDampingMoment;

    omegaDot << 
        ((parameter->rocket.inertiaRoll - parameter->rocket.inertiaRoll) * omega(1) * omega(2) + totalMoment(0)), 
        ((parameter->rocket.inertiaPitch - parameter->rocket.inertiaRoll) * omega(2) * omega(0) + totalMoment(1))/parameter->rocket.inertiaPitch,
        ((parameter->rocket.inertiaRoll - parameter->rocket.inertiaPitch) * omega(0) * omega(1) + totalMoment(2))/parameter->rocket.inertiaPitch;

    omegaDotMatrix <<
         0       ,-omega(0),-omega(1),-omega(2),
         omega(0), 0       , omega(2),-omega(1),
         omega(1),-omega(2), 0       , omega(0),
         omega(2), omega(1),-omega(0), 0;

    quatDot = 0.5 * omegaDotMatrix*quat;

    if(launcheCrear==false){
        omegaDot << 0,0,0;
        quatDot << 0,0,0,0;
    }

    dx << velEnu,accEnu,omegaDot,quatDot,massDot;
}

double Dynamics::interp1d(Eigen::MatrixXd inputMatrix, double time){
    double outputValue;
    double gradient;

    if(time < inputMatrix(0,0)){
        return inputMatrix(1,0);
    }
    else if(time >= inputMatrix(0,inputMatrix.cols()-1)){
        return inputMatrix(1,inputMatrix.cols()-1);
    }
    else{
        for(int index=0; index<inputMatrix.cols()-1; index++){
            if(inputMatrix(0,index) <= time && time < inputMatrix(0,index+1)){
                gradient = (inputMatrix(1,index+1) - inputMatrix(1,index)) / (inputMatrix(0,index+1) - inputMatrix(0,index));
                outputValue = gradient*(time - inputMatrix(0,index)) + inputMatrix(1,index);

                return outputValue;
            }
        }
        return outputValue;
    }
}
