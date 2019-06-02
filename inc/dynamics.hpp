#ifndef __DYNAMICS_HPP__
#define __DYNAMICS_HPP__

#include "parameter.hpp"

class Dynamics{
    public:
        Dynamics(Parameter *inputparameter);
        Dynamics();
        using state = Eigen::Matrix<double,14,1>;
        void operator()(const state &x, state &dx, const double t);

    private:
        Parameter *parameter;
        double interp1d(Eigen::MatrixXd inputMatrix, double time);
};

#endif