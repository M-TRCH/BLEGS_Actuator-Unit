
#ifndef IK_2DOF_H
#define IK_2DOF_H

#include <Arduino.h>
#include <Math.h>

class IK2DOF 
{
public:
    IK2DOF(float link1Length, float link2Length);
    void setTarget(float x, float y, bool elbow_up=true);
    void getJointAngles(float &theta1, float &theta2);
    void getJointAnglesDeg(float &theta1, float &theta2);

private:
    bool elbow_up;
    float link1Length;
    float link2Length;
    float targetX;
    float targetY;
};

#endif