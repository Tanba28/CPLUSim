#define _USE_MATH_DEFINES

#include <iostream>
#include <Eigen/Core>
#include <boost/numeric/odeint.hpp>
#include <cmath>

#include "dynamics.hpp"
#include "coordinate.hpp"

Dynamics::Dynamics(RocketParameter rocketIn,Enviroment envIn)
    : rocket(rocketIn),env(envIn)
{
}

void Dynamics::operator()(const state &x, state &dx, const double t){
    using namespace Eigen;
    using namespace std;

    Vector3d posEnu;
    Vector3d velEnu;
    Vector3d omega;
    Vector4d quat;

    Matrix3d dcm;
    Vector3d accEnu;
    Vector3d omegaDot;
    Matrix4d omegaDotMatrix;
    Vector4d quatDot;

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

    quat.normalized();

    dcm = Coordinate::convertQuatToDcm(quat);

    launcheCrear = (posEnu.norm() > env.railLength) && (posEnu(2) > 0);

    env.atmosphere(posEnu(2),temp,press,rho);
    g << 0,0,-env.g0;

    velWind = env.windLaw(posEnu(2));
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

    if(t < rocket.thrustTime){
        thrustForce = rocket.thrust;
    }
    else{
        thrustForce = 0;
    }

    dragForce = 0.5 * rho * (velAirNorm*velAirNorm) * rocket.area * rocket.cd;
    sideForce = 0.5 * rho * (velAirNorm*velAirNorm) * rocket.area * rocket.cna * beta;
    normalForce = 0.5 * rho * (velAirNorm*velAirNorm) * rocket.area * rocket.cna * alpha;

    totalForce << thrustForce - dragForce*cos(beta)*cos(alpha) ,sideForce, -normalForce;


    accEnu = (dcm.transpose()*(totalForce/rocket.mass)) + g;


    if((dcm*accEnu)(0) < 0 && launcheCrear == false){
        accEnu << 0,0,0;
    }

    aeroMoment << 
        0, 
        -normalForce*(rocket.lcp-rocket.lcg),
        -sideForce*(rocket.lcp-rocket.lcg);

    aeroDampingMoment <<
        0.25 * rho * velAirNorm * rocket.area * rocket.diameter*rocket.diameter * rocket.clp * omega(0),
        0.25 * rho * velAirNorm * rocket.area * rocket.length*rocket.length * rocket.cmq * omega(1),
        0.25 * rho * velAirNorm * rocket.area * rocket.length*rocket.length * rocket.cmq * omega(2);
    
    // ToDo:jetDampingMoment

    totalMoment = aeroMoment + aeroDampingMoment;

    omegaDot << 
        ((rocket.inertiaRoll - rocket.inertiaRoll) * omega(1) * omega(2) + totalMoment(0)), 
        ((rocket.inertiaPitch - rocket.inertiaRoll) * omega(2) * omega(0) + totalMoment(1))/rocket.inertiaPitch,
        ((rocket.inertiaRoll - rocket.inertiaPitch) * omega(0) * omega(1) + totalMoment(2))/rocket.inertiaPitch;

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

    dx << velEnu,accEnu,omegaDot,quatDot;
}
