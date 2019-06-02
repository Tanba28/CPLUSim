#include "enviroment.hpp"

Enviroment::Enviroment(picojson::object json){
    picojson::object envObj;
    envObj = json["Launch Pad Enviroment"].get<picojson::object>();
    velWind0 = envObj["Wind Velocity [m/s]"].get<double>();
    dirWind0 = envObj["Wind Direction [deg]"].get<double>();
    baseHeight = envObj["Wind Base Height [m]"].get<double>();
    windPower = envObj["Coefficient of Wind Power"].get<double>();

    azimuth0 = envObj["Launcher Azimuth [deg]"].get<double>();
    elevation0 = envObj["Launcher Elevation [deg]"].get<double>();

    railLength = envObj["Launcher Rail Length [m]"].get<double>();
}

Eigen::Vector3d Enviroment::windLaw(double height){
    Eigen::Vector3d velWind;
    double velWindAbs;
    if(height > 0.0){
        velWindAbs = velWind0 * pow((height / baseHeight),(1/windPower));
    }
    else{
        velWindAbs = 0;
    }

    velWind(0) = -velWindAbs * cos(deg2rad(dirWind0));
    velWind(1) = -velWindAbs * sin(deg2rad(dirWind0));
    velWind(2) = 0.0;

    return velWind;
}
void Enviroment::atmosphere(double height, double &temp, double &press, double &rho){
    double tempd = 0.0065;
    temp = temp0 - tempd*height;
    press = press0 * pow((temp/temp0),(g0/tempd/R));
    rho = press/(R*temp);
}
