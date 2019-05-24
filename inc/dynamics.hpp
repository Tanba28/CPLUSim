#ifndef __DYNAMICS_HPP__
#define __DYNAMICS_HPP__

#include "rocket.hpp"
#include "enviroment.hpp"

class Dynamics{
    public:
        Dynamics(RocketParameter rocketIn,Enviroment envIn);
        using state = Eigen::Matrix<double,13,1>;
        void operator()(const state &x, state &dx, const double t);

    private:
        RocketParameter rocket;
        Enviroment env;
};

#endif