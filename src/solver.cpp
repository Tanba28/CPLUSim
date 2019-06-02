#include "solver.hpp"

Solver::Solver(picojson::object json):
    parameter(json),
    dynamics(&parameter){
}

Dynamics::state Solver::intializeOdeVariable(){
    Dynamics::state initialVariable;

    Eigen::Vector3d euler;
    Eigen::Vector4d quaternion;
    Eigen::Matrix3d dcm;
    Eigen::Vector3d posEnu0;

    euler << 0, deg2rad(parameter.enviroment.elevation0), deg2rad(parameter.enviroment.azimuth0);
    dcm = Coordinate::convertEulerToDcm(euler);
    quaternion = Coordinate::convertDcmToQuat(dcm);

    posEnu0 << parameter.rocket.length - parameter.rocket.structureCG,0,0;
    posEnu0 = Coordinate::convertQuatToDcm(quaternion).transpose() * posEnu0;

    initialVariable << posEnu0, Eigen::VectorXd::Zero(6), quaternion, parameter.rocket.mass0;
    return initialVariable;
}

void Solver::launchSimulation(){
    Dynamics::state state0;

    state0 = intializeOdeVariable();
    Observer observer;

    boost::numeric::odeint::runge_kutta_dopri5<Dynamics::state,double,Dynamics::state,double,boost::numeric::odeint::vector_space_algebra> stepper;
    boost::numeric::odeint::integrate_const(stepper,dynamics,state0,0.0,50.0,0.01,observer);
}

void Observer::operator()(const Dynamics::state &x, const double t){
    std::cout << t << "," << x(0) << "," << x(1) << "," << x(2) << "," << x(3) << "," << x(4) << "," << x(5) <<  "," << x(13) << std::endl;
}