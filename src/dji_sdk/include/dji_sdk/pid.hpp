#ifndef _PID_H_
#define _PID_H_

//class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double dt, double max, double min, double Kp1, double Ki1, double Kd1,double Kp2, double Ki2, double Kd2 );
	PID();

        // Returns the manipulated variable given a setpoint and current process value
        double calculate(  double setpoint1, double setpoint2, double pv1, double pv2 );
        ~PID();

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp1;
    	double _Kd1;
    	double _Ki1;
	double _Kp2;
    	double _Kd2;
    	double _Ki2;
    	double _pre_error1;
    	double _pre_error2;
    	double _integral1;
    	double _integral2;
};

#endif
