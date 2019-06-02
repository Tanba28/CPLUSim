#ifndef __PARAMETER_HPP__
#define __PARAMETER_HPP__

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

#include "rocket.hpp"
#include "enviroment.hpp"
#include "picojson.h"

class Parameter{
    public:
        Parameter(picojson::object json);

        Rocket rocket;
        Enviroment enviroment;

    private:
        
};


#endif