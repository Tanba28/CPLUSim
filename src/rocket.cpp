#include "rocket.hpp"

Rocket::Rocket(picojson::object json){
    picojson::object systemObj;
    picojson::object mechanicsObj;
    picojson::object aeroObj;
    picojson::object recoveryObj;
    picojson::object motorObj;

    systemObj = json["System"].get<picojson::object>();
    thrustFileName = systemObj["Thrust File Name"].get<std::string>();
    rocketName = systemObj["Name"].get<std::string>();
    modelRocket = systemObj["ModelRocket"].get<bool>();

    mechanicsObj = json["Mechanics"].get<picojson::object>();
    length = mechanicsObj["Length [m]"].get<double>();
    diameter = mechanicsObj["Diameter [m]"].get<double>();
    area = M_PI_4 * diameter * diameter;
    structureMass = mechanicsObj["Structure Mass [kg]"].get<double>();
    structureCG = mechanicsObj["Structure CG From Nose [m]"].get<double>();
    inertiaPitch = mechanicsObj["Structure Inertia Moment(pitch/yaw) [kg*m^2]"].get<double>();
    inertiaRoll = mechanicsObj["Structure Inertia Moment(roll) [kg*m^2]"].get<double>();

    aeroObj = json["AeroDynamics"].get<picojson::object>();
    lcp = aeroObj["CP From Nose [m]"].get<double>();
    clp = aeroObj["Coefficient of Damping Moment (Roll)"].get<double>();
    cmq = aeroObj["Coefficient of Damping Moment (Pitch/Yaw)"].get<double>();
    cd = aeroObj["Coefficient of Drag"].get<double>();
    cna = aeroObj["Coefficient of Normal Force"].get<double>();

    //TODO
    recoveryObj = json["Recovery"].get<picojson::object>();
    deployTimePara1 = recoveryObj["1st Parachute Deploy Time [s]"].get<double>();
    decentVelPara1 = recoveryObj["Decent Speed of 1st Parachute[m/s]"].get<double>();
    parachute2 = recoveryObj["2nd Parachute Exist"].get<bool>();
    deployAltitudePara2 = recoveryObj["2nd Parachute Deploy Altitude [m]"].get<double>();
    decentVelPara2 = recoveryObj["Decent Speed of 2nd Parachute[m/s]"].get<double>();

    if(modelRocket == true){
        motorObj = json["ModelRocket Engine"].get<picojson::object>();
        isp = motorObj["Isp [s]"].get<double>();
        motorLength = motorObj["Motor Length [m]"].get<double>();
        motorDiameter = motorObj["Moter Diameter [m]"].get<double>();
        motorCG = motorObj["Motor CG From Nose [m]"].get<double>();
        motorMassBefore = motorObj["Motor Mass Before Combustion [kg]"].get<double>();
        motorMassAfter = motorObj["Motor Mass After Combustion [kg]"].get<double>();
    }
    else{
        motorObj = json["Hybrid Engine"].get<picojson::object>();
        isp = motorObj["Isp [s]"].get<double>();
        oxidizerTankLength = motorObj["Oxidizer Tank Length [m]"].get<double>();
        oxidizerTankCG = motorObj["Oxidizer Tank CG From Nose [m]"].get<double>();
        oxidizerMass = motorObj["Oxidizer Mass [kg]"].get<double>();
        oxidizerMassDot = motorObj["Oxidizer Mass Flow Rate [kg/s]"].get<double>();
        fuelLength = motorObj["Fuel Length [m]"].get<double>();
        fuelCG = motorObj["Fuel CG From Nose [m]"].get<double>();
        fuelMassBefore = motorObj["Fuel Mass Before Combustion [kg]"].get<double>();
        fuelMassAfter = motorObj["Fuel Mass After Combustion [kg]"].get<double>();
        fuelMassDot = motorObj["Fuel Mass Flow Rate [kg/s]"].get<double>();
    }

    mass0 = structureMass + oxidizerMass + fuelMassBefore;

    //TODO 関数にする
    std::ifstream fs;
    std::string line;
    std::stringstream stream;
    std::string temp;
    int col=0;
    fs.open(thrustFileName);
    while(std::getline(fs,line)){
        thrust.conservativeResize(2, col+1);
        std::stringstream stream(line);
        for(int row=0;row<2;row++){
            std::getline(stream,temp,',');
            thrust(row,col) = std::stod(temp);
        }
        col++;
    }

    thrustTime = thrust(0,thrust.cols()-1);
    fs.close();
}