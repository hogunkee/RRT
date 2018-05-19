#include "ros/ros.h"
#include <stdio.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hello");
	ros::NodeHandle n;
	printf("hello!! World!!\n");
	ros::spinOnce();

	return 0;
}
