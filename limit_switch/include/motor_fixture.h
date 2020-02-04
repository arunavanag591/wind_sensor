#ifndef MOTOR_FIXTURE
#define MOTOR_FIXTURE

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

using namespace std; 

class MotorFixture{
    private:

    public:
    MotorFixture();
    ~MotorFixture();
    void Initialize(ros::NodeHandle&);
    
};

#endif