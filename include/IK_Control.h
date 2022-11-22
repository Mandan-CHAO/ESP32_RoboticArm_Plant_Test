#ifndef IK_Control_h
#define IK_Control_h
#include <Arduino.h>
#include "Constants.h"
#include "MoveBatch.h"

struct Point3D
{
    double x; 
    double y;
    double z;
};

class IK_Control
{
    public:
    IK_Control(double base0, double link1, double link2, double link3);
    MoveBatch runIKControl(double x, double y, double z, MoveBatch mb, double theta = 0.0);

    private:
        double _base0, _link1, _link2, _link3;
        double EnderAngle;
        double _alpha, _beta, _gamma, _theta;
};

#endif