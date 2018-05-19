#include <project1/pid.h>

PID::PID(){

	error = 0.0f;
	error_sum = 0.0f;
	error_diff = 0.0f;
	Kp = 0.6f;
	Ki = 0.001f;
	Kd = 0.05f;
	W_g1 = 0.7f;
	W_g2 = 0.5f;

}
float PID::get_distance(point car_pose, point goal_pose){
	return sqrt(pow(car_pose.x - goal_pose.x, 2) + pow(car_pose.y - goal_pose.y, 2));
}
void PID::init_errorsum(){
	error_sum=0.0f;
}
float PID::get_control(point car_pose, point goal_pose){

    float ctrl;
	float pre_error = error;
	float th_g, distance;
	float MAX_STEER = 3.0;	//max steering angular
	float MAX_ES = 10;	//max error_sum (clipping)

	distance = get_distance(car_pose, goal_pose);
	th_g = atan((car_pose.y - goal_pose.y)/(car_pose.x - goal_pose.x));

	if (th_g < 0 && car_pose.x - goal_pose.x >0)
		th_g += M_PI;
	else if (th_g > 0 && car_pose.x - goal_pose.x >0)
		th_g -= M_PI;

	if (distance > 0.5)
		error = th_g - car_pose.th;
	else if (distance > 0.3)
		error = W_g1*th_g + (1-W_g1)*goal_pose.th - car_pose.th;
	else
		error = W_g2*th_g + (1-W_g2)*goal_pose.th - car_pose.th;
	error_sum += error;
	error_diff = error - pre_error; 

	//integral windup - clipping
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

