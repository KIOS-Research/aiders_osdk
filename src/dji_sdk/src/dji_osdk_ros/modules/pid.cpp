#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include <dji_sdk/pid.hpp>

using namespace std;

PID::PID( double dt, double max, double min, double Kp1, double Ki1, double Kd1,double Kp2, double Ki2, double Kd2  )
{
    _dt=dt;
    _max=max;
    _min=min;
    _Kp1=Kp1;
    _Kd1=Kd1;
    _Ki1=Ki1;
    _Kp2=Kp2;
    _Kd2=Kd2;
    _Ki2=Ki2;
    _pre_error1=0;
    _pre_error2=0;	
    _integral1=0;
    _integral2=0;
}

PID::PID(){}

double PID::calculate( double setpoint1, double setpoint2, double pv1, double pv2 )
{
    //return pimpl->calculate(setpoint,pv);
    // Calculate error
    double error1 = setpoint1 - pv1;
    double error2 = setpoint2 - pv2;
    
    // Proportional term
    double Pout = _Kp1 * error1 + _Kp2 * error2;

    // Integral term
    _integral1 += error1 * _dt ;
    _integral2 += error2 * _dt ;
    double Iout = _Ki1 * _integral1 + _Ki2 * _integral2;

    // Derivative term
    double derivative1 = (error1 - _pre_error1) / _dt;
    double derivative2 = (error2 - _pre_error2) / _dt;	
    double Dout = _Kd1 * derivative1 + _Kd2 * derivative2;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error1 = error1;
    _pre_error2 = error2;
    return output;
}
PID::~PID() 
{
}

#endif
