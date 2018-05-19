#include <cmath>
#ifndef POINT_H
#define POINT_H
#include <project2/point.h>
#include <project2/traj.h>
#endif

class PID{
public:
    PID();

    //this function makes control output using arguments which are the current value and the target setpoint.
    float get_control(point car_pose, traj prev_goal, traj cur_goal);
	float get_distance(point car_pose, traj goal);
	void init_errorsum();
private:
    float error;
    float error_sum;
    float error_diff;
    float Kp;
    float Ki;
    float Kd;
	float W_g1;
	float W_g2;
};
