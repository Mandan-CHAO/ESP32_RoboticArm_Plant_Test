#include <Arduino.h>
#include "IK_Control.h"
#include "Constants.h"
#include "MoveBatch.h"

IK_Control::IK_Control(double base0, double link1, double link2, double link3)
{
    _base0 = base0;
    _link1 = link1;
    _link2 = link2;
    _link3 = link3;

    pinMode(NAN_ALERT_LED, OUTPUT); 
}


MoveBatch IK_Control(double x, double y, double z, MoveBatch mb, double theta=0)
{
    Point3D _P_0, _P_1, _P_2, _P_3, _P_4;
    double _theta, _alpha, _beta, _gamma;

    _theta = atan(y/x);
    
    double dis = sqrt(x*x + y*y);
    
    
    mb.addMove(/*id:*/ 0, /*pos:*/ (int32_t)(PULSES_PER_REVOLUTION * 2* _theta / M_PI));
    mb.addMove(/*id:*/ 1, /*pos:*/ (int32_t)(PULSES_PER_REVOLUTION * _alpha / M_PI));
    mb.addMove(/*id:*/ 2, /*pos:*/ (int32_t)(PULSES_PER_REVOLUTION * _beta / M_PI));
    mb.addMove(/*id:*/ 3, /*pos:*/ (int32_t)(PULSES_PER_REVOLUTION * _gamma / M_PI));

    return mb;
    
}