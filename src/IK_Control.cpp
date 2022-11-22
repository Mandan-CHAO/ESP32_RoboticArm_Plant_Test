#include <Arduino.h>
#include "IK_Control.h"
#include "Constants.h"
#include "MoveBatch.h"

IK_Control::IK_Control(double base0, double link1, double link2, double link3)
{
    _base0 = base0;  //55.30mm
    _link1 = link1;  //83.00mm
    _link2 = link2;  //83.00mm
    _link3 = link3;  //49.50mm

    pinMode(NAN_ALERT_LED, OUTPUT); 
}


MoveBatch IK_Control::runIKControl(double x, double y, double z, MoveBatch mb, double theta)
{
    Point3D _P_0, _P_1, _P_2, _P_3, _P_4;
    double _theta, _alpha, _beta, _gamma;
    EnderAngle = theta;

    //Assume that the x- and y-axis are in the horizontal plane
    //  postive x-direction is along the right direction
    //  postive y-direction is the user-front directioin 
    //  postive z-direction is up vertical direction

    // For the relation of steps to rad, half-step: 2048-PI full-step:1024-PI
    // postive direction: Motor1:cw Motor2:

    //120 90 30
    
    if (x > 0){
        _theta = atan(y/x);
    }else if (x < 0){
        _theta = atan(y/x) + M_PI;
    }else{
        if (y > 0){
            _theta = M_PI_2;
        }else if (y < 0)
        {
            _theta = -M_PI_2;
        }else{
            _theta = 0.0;
        }
    }
    
    double dis = sqrt(x*x + y*y);
    double delta_x = dis - _link3*cos(EnderAngle);
    double delta_z = z + _link3*sin(EnderAngle) - _base0;

    double M = sqrt(delta_x*delta_x + delta_z*delta_z);
    double a = acos((_link1*_link1 + _link2*_link2 - M*M)/(2*_link1*_link2));
    _beta = a - M_PI;

    double c = atan(delta_z/delta_x);
    _alpha = (M_PI - a)/2 + c - M_PI_2;
    
    _gamma = EnderAngle - M_PI_2 + _alpha + a;
    
    mb.addMove(/*id:*/ 0, /*pos:*/ (int32_t)(-2048 * _theta / M_PI));
    mb.addMove(/*id:*/ 1, /*pos:*/ (int32_t)(1024 * _alpha / M_PI));
    mb.addMove(/*id:*/ 2, /*pos:*/ (int32_t)(1024 * _beta / M_PI));
    mb.addMove(/*id:*/ 3, /*pos:*/ (int32_t)(1024 * _gamma / M_PI));


    return mb;
    
}