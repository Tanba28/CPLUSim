#include "parameter.hpp"

Parameter::Parameter(picojson::object json):
    rocket(json),
    enviroment(json){
}