#include <cmath>
#ifndef POINT_H
    #define POINT_H
    #include <project1/point.h>
#endif

class PID{
public:
    PID();

    //this function makes control output using arguments which are the current value and the target setpoint.
	float get_distance(point car_pose, point goal_pose);
	void init_errorsum();
    float get_control(point car_pose, point goal_pose);

private:

    // you can use this private member variables or additionally define other member variables as you want.
    float error;
    float error_sum;
    float error_diff;
    float Kp;
    float Ki;
    float Kd;
	float W_g1; //weight of th_g
	float W_g2; //weight of th_g
};
