#define _USE_MATH_DEFINES

#include <iostream>
#include <Eigen/Core>
#include <cmath>

#include "coordinate.hpp"


namespace Coordinate{
    Eigen::Matrix3d convertQuatToDcm(Eigen::Vector4d quat){
        Eigen::Matrix3d dcm;
        dcm << 
            quat(0)*quat(0)+quat(1)*quat(1)-quat(2)*quat(2)-quat(3)*quat(3), 2*(quat(1)*quat(2)+quat(0)*quat(3))                            , 2*(quat(1)*quat(3)-quat(0)*quat(2)),
            2*(quat(1)*quat(2)-quat(0)*quat(3))                            , quat(0)*quat(0)-quat(1)*quat(1)+quat(2)*quat(2)-quat(3)*quat(3), 2*(quat(2)*quat(3)+quat(0)*quat(1)),
            2*(quat(1)*quat(3)+quat(0)*quat(2))                            , 2*(quat(2)*quat(3)-quat(0)*quat(1))                            , quat(0)*quat(0)-quat(1)*quat(1)-quat(2)*quat(2)+quat(3)*quat(3);
        
        return dcm;
    }

    Eigen::Matrix3d convertEulerToDcm(Eigen::Vector3d euler){
        double phi = euler[0];
        double theta = euler[1];
        double psi = euler[2];
        Eigen::Matrix3d dcm;
        dcm <<
             cos(theta)*cos(psi)                             , cos(theta)*sin(psi)                             , -sin(theta),
            -cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta),
             sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi),-sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta);

        return dcm;
    }

    Eigen::Vector3d convertQuatToEuler(Eigen::Vector4d quat){
        Eigen::Matrix3d dcm = Coordinate::convertQuatToDcm(quat);
        Eigen::Vector3d euler;
        euler << atan2(dcm(1,2),dcm(2,2)),-asin(dcm(0,2)),atan2(dcm(0,1),dcm(0,0));
        euler(0) = rad2deg(euler(0));
        euler(1) = rad2deg(euler(1));
        euler(2) = rad2deg(euler(2));

        return euler;
    }

    Eigen::Vector4d convertDcmToQuat(Eigen::Matrix3d dcm){
        Eigen::Vector4d quat;
        if((1.0 + dcm(0,0) + dcm(1,1) + dcm(2,2)) <= 0){
            quat(0)= 0;
        }
        else{
            quat(0) = 0.5 * sqrt(1.0 + dcm(0,0) + dcm(1,1) + dcm(2,2));

        }

        if((1.0 + dcm(0,0) - dcm(1,1) - dcm(2,2)) <= 0){
            quat(1) = 0;
        }
        else{
            quat(1) = 0.5 * sqrt(1.0 + dcm(0,0) - dcm(1,1) - dcm(2,2));
        }

        if((1.0 - dcm(0,0) + dcm(1,1) - dcm(2,2)) <= 0){
            quat(2) = 0;
        }
        else{
            quat(2) = 0.5 * sqrt(1.0 - dcm(0,0) + dcm(1,1) - dcm(2,2));
        }

        if((1.0 - dcm(0,0) - dcm(1,1) + dcm(2,2)) <= 0){
            quat(3) = 0;
        }
        else{
            quat(3) = 0.5 * sqrt(1.0 - dcm(0,0) - dcm(1,1) + dcm(2,2));
        }

        Eigen::VectorXd::Index quatMax;
        quat.maxCoeff(&quatMax);

        switch(quatMax){
            case 0:
                quat(0) = 0.5 * sqrt(1.0+dcm(0,0)+dcm(1,1)+dcm(2,2));
                quat(1) = (dcm(1,2)-dcm(2,1))/(4.0 * quat(0));
                quat(2) = (dcm(2,0)-dcm(0,2))/(4.0 * quat(0));
                quat(3) = (dcm(0,1)-dcm(1,0))/(4.0 * quat(0));
                break;

            case 1:
                quat(1) = 0.5 * sqrt(1.0+dcm(0,0)-dcm(1,1)-dcm(2,2));
                quat(2) = (dcm(0,1)+dcm(1,0))/(4.0*quat(1));
                quat(3) = (dcm(2,0)+dcm(0,2))/(4.0*quat(1));
                quat(0) = (dcm(1,2)-dcm(2,1))/(4.0*quat(1));
                break;
            
            case 2:
                quat(2) = 0.5 * sqrt(1.0-dcm(0,0)+dcm(1,1)-dcm(2,2));
                quat(1) = (dcm(0,1)+dcm(1,0))/(4.0*quat(2));
                quat(3) = (dcm(1,2)+dcm(2,1))/(4.0*quat(2));
                quat(0) = (dcm(2,0)-dcm(0,2))/(4.0*quat(2));
                break;
            
            case 3:
                quat(3) = 0.5 * sqrt(1.0-dcm(0,0)-dcm(1,1)+dcm(2,2));
                quat(1) = (dcm(2,0)+dcm(0,2))/(4.0*quat(3));
                quat(2) = (dcm(1,2)+dcm(2,1))/(4.0*quat(3));
                quat(0) = (dcm(0,1)-dcm(1,0))/(4.0*quat(3)); 
                break; 
        }
        
        quat.normalize();

        return quat;
    }
}