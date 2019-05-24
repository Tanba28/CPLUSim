#include <iostream>
#include <string>
#include <Eigen/Core>
#include <boost/numeric/odeint.hpp>

#include "rocket.hpp"
#include "enviroment.hpp"
#include "dynamics.hpp"
#include "coordinate.hpp"

void observer(const Dynamics::state &x, const double t){
    std::cout << t << "," << x(0) << "," << x(1) << "," << x(2) << "," << x(3) << "," << x(4) << "," << x(5) <<  std::endl;
}

int main(){
    RocketParameter rocket;
    Enviroment env;
    Dynamics dynamics(rocket,env);

    double phi,theta,psi;
    phi = 0;
    theta = deg2rad(env.elevation0);
    psi = deg2rad(env.azimuth0);

    Eigen::Vector3d euler = {phi,theta,psi};
    Eigen::Vector4d quat;
    Eigen::Matrix3d dcm;
    dcm = Coordinate::convertEulerToDcm(euler);
    quat = Coordinate::convertDcmToQuat(dcm);

    Eigen::Vector3d posEnu0 = {rocket.length - rocket.lcg,0,0};
    posEnu0 = Coordinate::convertQuatToDcm(quat).transpose() * posEnu0;

    Dynamics::state state0;
    state0 << posEnu0,0,0,0,0,0,0,quat;

    boost::numeric::odeint::runge_kutta_dopri5<Dynamics::state,double,Dynamics::state,double,boost::numeric::odeint::vector_space_algebra> stepper;

    boost::numeric::odeint::integrate_const(stepper,dynamics,state0,0.0,50.0,0.01,observer);
                           
    return 0;
}