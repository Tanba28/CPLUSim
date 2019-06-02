#ifndef __SOLVER_HPP__
#define __SOLVER_HPP__

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>

#include "picojson.h"
#include "parameter.hpp"
#include "dynamics.hpp"
#include "coordinate.hpp"

class Solver{
    public:
        Solver(picojson::object json);

        Dynamics::state intializeOdeVariable();
        void launchSimulation();

    private:
        Parameter parameter;
        Dynamics dynamics;
};

class Observer{
    public:
        void operator()(const Dynamics::state &x, const double t);
};
#endif