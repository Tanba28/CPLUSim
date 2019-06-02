#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <boost/numeric/odeint.hpp>

#include "picojson.h"
#include "solver.hpp"

int main(int argc, char *argv[]){
    std::cout << "CPLUSim" << "Ver0.01" << std::endl;

    if(argc < 2){
        std::cerr << "failed" <<std::endl;
        return 1;        
    }

    std::ifstream fs;

    fs.open(argv[1],std::ios::in);
    if(fs.fail()){
        std::cerr << "failed" <<std::endl;
        return 1;
    }
    picojson::value val;

    fs >> val;
    fs.close();

    Solver solver(val.get<picojson::object>());

    solver.launchSimulation();
                           
    return 0;
}