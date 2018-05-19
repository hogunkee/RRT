#include <project2/pid.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

PID::PID(){

    /* TO DO
     *
     * find appropriate values for PID contorl, and initialize them.
     *
    */

    error = 0.0f;
    error_sum = 0.0f;
    error_diff = 0.0f;
    Kp = 0.8;
    Ki = 0;
    Kd = -0.35;
	/*
    Kp = 0.35f;///0.45f;	//0.5f;//0.6f;
    Ki = 0.008f;///0.004f;	//0.0001f;
    Kd = -0.008f;///0.14f;	//0.10f; 
	*/
	W_g1 = 1.0f;
	W_g2 = 1.0f;
}

float PID::get_distance(point car_pose, traj goal){
	return sqrt(pow(car_pose.x - goal.x,2) + pow(car_pose.y - goal.y,2));
}
void PID::init_errorsum(){
	error_sum = 0.0f;
}
float PID::get_control(point car_pose, traj prev_goal, traj cur_goal) {
	float ctrl;
	float pre_error = error;
	float th_g, distance;
	float MAX_STEER = 1.0; //1.0;
	float MAX_ES = 10;

	distance = get_distance(car_pose, cur_goal);
	th_g = atan((car_pose.y - cur_goal.y)/(car_pose.x - cur_goal.x));
	if (th_g < 0 && car_pose.x - cur_goal.x > 0)
		th_g += PI;
	else if (th_g > 0 && car_pose.x - cur_goal.x > 0)
		th_g -= PI;

	error = th_g - car_pose.th;
	
	if (distance > 1.2)
		error = th_g - car_pose.th;
	else if  (distance > 0.8)
		error = W_g1*th_g + (1-W_g1)*cur_goal.th - car_pose.th;
	else
		error = W_g2*th_g + (1-W_g2)*cur_goal.th - car_pose.th;
	
	if (error < -PI)
		error += 2*PI;
	else if (error > PI)
		error -= 2*PI;

	error_sum += error;
	error_diff = error - pre_error;

	if (error_sum > MAX_ES)
		error_sum = MAX_ES;
	else if (error_sum < -MAX_ES)
		error_sum = -MAX_ES;

	ctrl = Kp * error + Ki * error_sum + Kd * error_diff;

	if (ctrl > MAX_STEER)
		ctrl = MAX_STEER;
	else if (ctrl < -MAX_STEER)
		ctrl = -MAX_STEER;

	return ctrl;
}

