
#include "ik_2dof.h"

IK2DOF::IK2DOF(float link1Length, float link2Length)
{
    this->link1Length = link1Length;
    this->link2Length = link2Length;
}

void IK2DOF::setTarget(float x, float y, bool elbow_up)
{
    this->targetX = x;
    this->targetY = y;
    this->elbow_up = elbow_up;
}

void IK2DOF::getJointAngles(float &theta1, float &theta2)
{
    float r = sqrt(targetX*targetX + targetY*targetY);
    float cos_theta2 = (r*r - link1Length*link1Length - link2Length*link2Length) / (2*link1Length*link2Length);

    // Clamp the value between -1 and 1
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

    theta2 = acos(cos_theta2);
    if (!elbow_up) 
    {
        theta2 = -theta2;
    }
    theta1 = atan2(targetY, targetX) - atan2(link2Length*sin(theta2), link1Length + link2Length*cos(theta2));
}

void IK2DOF::getJointAnglesDeg(float &theta1, float &theta2)
{
    getJointAngles(theta1, theta2);
    theta1 *= RAD_TO_DEG;
    theta2 *= RAD_TO_DEG;
}