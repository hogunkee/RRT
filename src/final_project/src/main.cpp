//state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING 2
#define FINISH -1
#define PI 3.14159265358979323846

#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

//map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//parameters we should adjust : K, margin, MaxStep
int margin = 5.0;//4
int K = 3000;
double MaxStep = 0.8;//0.8 / 1.0
int waypoint_margin = 24;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;

//FSM state
int state;

//function definition
void setcmdvel(double v, double w);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT();
ros::Publisher pathPublisher;

nav_msgs::Path Construct_Path_Msg(double* x, double* y, int length){
	nav_msgs::Path msg;
	msg.header.frame_id = "map";
	std::vector<geometry_msgs::PoseStamped> poses(length);
	for (int i=0; i<length; i++){
		poses.at(i).pose.position.x = x[i];
		poses.at(i).pose.position.y = y[i];
	}
	msg.poses = poses;
	return msg;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "slam_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);
	pathPublisher = n.advertise<nav_msgs::Path>("/RRTpath",1);	//TODO rosrun project 하고 rviz에서 RRTpath 선택. 메시지 넣는건 구글링
    ros::Subscriber gazebo_pose_sub = n.subscribe("/amcl_pose", 100, callback_state);

    printf("Initialize topics\n");
    
    // FSM
    state = INIT;
    bool running = true;
    ros::Rate control_rate(60);
	PID pid_ctrl;
    int look_ahead_idx;
	int waypoint_idx;
	int count = 1;

    while(running){
        switch (state) {
        case INIT: {
            // Load Map
            char* user = getpwuid(getuid())->pw_name;
            cv::Mat map_org = cv::imread((std::string("/home/") + std::string(user) +
                              std::string("/catkin_ws/src/final_project/src/final.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.7;
            world_x_max = 4.7;
            world_y_min = -10.2;
            world_y_max = 10.2;
            res = 0.05;
            printf("Load map\n");

             if(! map.data )                              // Check for invalid input
            {
                printf("Could not open or find the image\n");
                return -1;
            }
            state = PATH_PLANNING;
        } break;

        case PATH_PLANNING:
            
            // Set Way Points
            set_waypoints();
            printf("Set way points\n");

            // RRT
            generate_path_RRT();
            printf("Generate RRT\n");

            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");

            look_ahead_idx = 0;
			waypoint_idx = 1;
			printf("path size : %d\n", path_RRT.size());

            state = RUNNING;

        case RUNNING: {
			float dist = pid_ctrl.get_distance(robot_pose, path_RRT[look_ahead_idx]);
			if (dist < 1.7){
				if (waypoints.at(waypoint_idx).x == path_RRT[look_ahead_idx].x && waypoints.at(waypoint_idx).y == path_RRT[look_ahead_idx].y){
					if (dist < 0.5){
						printf("-----------------------------------------\n");
						printf("Arrive at the %d-th Way Point!! %d times costed\n", waypoint_idx, count);
						printf("-----------------------------------------\n");
						waypoint_idx++;
						if (waypoint_idx == waypoints.size()){
							state = FINISH;
							break;
						}
						look_ahead_idx++;
					}
				} else{
					printf("-----------------------------------------\n");
					printf("pass the goal: %d, cost: %d times\n", look_ahead_idx, count);
					printf("-----------------------------------------\n");
					look_ahead_idx++;
				}
			}
			traj prev_g = path_RRT[look_ahead_idx - 1];
			traj cur_g = path_RRT[look_ahead_idx];
			float ctrl = pid_ctrl.get_control(robot_pose, prev_g, cur_g);
			//double speed = (1.0 - fabs(ctrl))*2.0 + 1.0;
			double speed = (1.0 - fabs(ctrl))*1.5 + 1.0;
			//if ((robot_pose.x > 3 || robot_pose.x < -3) && (robot_pose.y < 4.5 && robot_pose.y > -4.5)){

			if ((robot_pose.x > 3 && robot_pose.y > -3.3 && robot_pose.y < 5.0) || (robot_pose.x < -3 && robot_pose.y < 3.3 && robot_pose.y > -5.0)){
				//speed = (0.9 - fabs(ctrl))*0.8 + 1.0;
				speed = abs(speed) > 1.8 ? 1.8 : speed;
				//ctrl += 0.05;
				if ((robot_pose.x > 3 && robot_pose.y > 4.0 && robot_pose.y < 4.2) || (robot_pose.x < -3 && robot_pose.y < -4.0 && robot_pose.y > -4.2)){
					if (abs(robot_pose.x > 3.7))
						ctrl = -0.10;
				}
				/*
				if ((robot_pose.x > 3 && robot_pose.y > -2.5 && robot_pose.y < 4.0) || (robot_pose.x < -3 && robot_pose.y < -3.3 && robot_pose.y > 2.5)){
					if (abs(robot_pose.x) < 3.5)
						ctrl = 0.05;
					else
						ctrl = -0.05;
				}
				if ((robot_pose.x > 3 && robot_pose.y > -6 && robot_pose.y < -3.0) || (robot_pose.x < -3 && robot_pose.y < 6 && robot_pose.y > 3.0)){
					if (abs(robot_pose.x) > 3.8){
						ctrl = -0.15;
					}
				}
				*/
			}

			setcmdvel(speed, ctrl);
			//setcmdvel(((0.8 - fabs(ctrl))*2 + 1), ctrl);
			//setcmdvel(((M_PI - fabs(ctrl)) + 0.5), ctrl);
			//setcmdvel(((M_PI - fabs(ctrl)) + 1.0), ctrl);

			//setcmdvel(((M_PI - fabs(ctrl))/2 + 1.0), ctrl);
			//setcmdvel(2.0, ctrl);
            cmd_vel_pub.publish(cmd);
			ros::spinOnce();
			control_rate.sleep();
			printf("ctrl(steering): %.2f velocity: %.2f\n", ctrl, speed);
			printf("car pose: %.2f,%.2f,%.2f\n", robot_pose.x, robot_pose.y, robot_pose.th);
			printf("from: %.1f, %.1f to: %.1f, %.1f\n", robot_pose.x, robot_pose.y, cur_g.x, cur_g.y);
			count++;
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd);
            running = false;
            ros::spinOnce();
            control_rate.sleep();
        } break;
        default: {
        } break;
        }
    }
    return 0;
}

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs){
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
    //printf("x,y : %f,%f \n",robot_pose.x,robot_pose.y);
}

void set_waypoints()
{
    point waypoint_candid[7];

    // Starting point. (Fixed)
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 8.5;


    //TODO 2
    // Set your own waypoints.
    // The car should turn around the outer track once, and come back to the starting point.
    // This is an example.
	/* 11111
    waypoint_candid[1].x = -1.0;
    waypoint_candid[1].y = 8.8;
	*/
    waypoint_candid[1].x = -0.5;//1.0
    waypoint_candid[1].y = 9.0;//9.0
    //waypoint_candid[1].y = 9.0;
    waypoint_candid[1].th = -0.2;//-0.2
    waypoint_candid[2].x = 3.5;
    waypoint_candid[2].y = 6.6;//6.7
    waypoint_candid[2].th = - PI * 1/3;
	/*
    waypoint_candid[2].x = 3.8;
    waypoint_candid[2].y = 5.5;
	*/
    waypoint_candid[3].x = -3.3;//-3.7;
    waypoint_candid[3].y = -6.8;//-6.7;
	waypoint_candid[3].th = PI*2/3;//PI * 2/3;
	/*
    waypoint_candid[3].x = -2.5;
    waypoint_candid[3].y = -8.3;
	*/
    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 8.5;
	waypoint_candid[4].th = PI * 1/3;
	/*
    waypoint_candid[1].x = 2.2;
    waypoint_candid[1].y = 8.5;
    waypoint_candid[2].x = 2.5;
    waypoint_candid[2].y = -8.5;
    waypoint_candid[3].x = -2.5;
    waypoint_candid[3].y = -8.0;
    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 8.5;
	*/


    // Waypoints for arbitrary goal points.
    // TA will change this part before scoring.
    // This is an example.
    waypoint_candid[5].x = 1.5;
    waypoint_candid[5].y = 1.5;
    waypoint_candid[6].x = -2;
    waypoint_candid[6].y = -9.0;

    int order[] = {0,1,2,3,4,5,6};
    int order_size = 7;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void generate_path_RRT()
{   
	waypoints.at(0).th = robot_pose.th;
	rrtTree tmp_rrt;
	for(int i = 0; i < waypoints.size()-1; i++){
		tmp_rrt = rrtTree(waypoints[i], waypoints[i+1], map, map_origin_x, map_origin_y, res, margin);
		tmp_rrt.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
		//tmp_rrt.visualizeTree();	//visualize tree
		std::vector<traj> tmp_path = tmp_rrt.backtracking_traj();
		printf("%d-th waypoint: %.2f, %.2f\n", i+1, tmp_path.at(0).x, tmp_path.at(0).y);
		printf("theta: %.2f\n", tmp_path.at(0).th);
		waypoints[i+1].th=tmp_path.at(0).th;
		//tmp_rrt.visualizeTree(tmp_path);	//visualize path
		std::reverse(tmp_path.begin(), tmp_path.end());
		path_RRT.insert(path_RRT.end(), tmp_path.begin(), tmp_path.end());
	}

	double _x[path_RRT.size()];
	double _y[path_RRT.size()];
	for (int i=0; i<path_RRT.size(); i++){
		_x[i] = path_RRT.at(i).x;
		_y[i] = path_RRT.at(i).y;
	}
	pathPublisher.publish(Construct_Path_Msg(_x, _y, path_RRT.size()));
	//tmp_rrt.visualizeTree(path_RRT);
}
