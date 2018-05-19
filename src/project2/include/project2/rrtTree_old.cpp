#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#include <math.h>
#define PI 3.14159265358979323846

double max_alpha = 0.2;
double L = 0.325;


rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal) {
    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
}

rrtTree::~rrtTree(){
    for (int i = 1; i <= count; i++) {
        delete ptrTable[i];
    }
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);
    
    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	for(int j = 0; j < 10; j++) {
	    double alpha = this->ptrTable[i]->alpha;
	    double d = this->ptrTable[i]->d;
	    double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	    double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	    double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }
    cv::namedWindow("Mapping");
	cv::imshow("Mapping", imgResult);
    //cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    //cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	for(int j = 0; j < 10; j++) {
	    double alpha = this->ptrTable[i]->alpha;
	    double d = this->ptrTable[i]->d;
	    double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	    double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	    double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
	for(int j = 0; j < 10; j++) {
	    double alpha = path[i].alpha;
	    double d = path[i].d;
	    double p1_th = path[i-1].th + d*j/10*tan(alpha)/L; // R = L/tan(alpha)
            double p2_th = path[i-1].th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = path[i-1].x + L/tan(alpha)*(sin(p1_th) - sin(path[i-1].th));
	    double p1_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p1_th));
            double p2_x = path[i-1].x + L/tan(alpha)*(sin(p2_th) - sin(path[i-1].th));
	    double p2_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }
    cv::namedWindow("Mapping");
	cv::imshow("Mapping", imgResult);
    //cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    //cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d) {
    //TODO
    if(alpha < max_alpha || alpha > -max_alpha){
        node* new_node = new node;
        new_node->idx = count; 
        new_node->idx_parent = idx_near;
        new_node->location = x_new;
        new_node->rand = x_rand;
        new_node->alpha = alpha;
        new_node->d = d;
        ptrTable[count] = new_node;
        count += 1;
    }
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
        //TODO
    double out[5];
    for(int i = 0; i < K; i++){
        point x_rand = randomState( x_max, x_min, y_max, y_min);
        if (i%4 == 0){
            x_rand = x_goal; 
        }
        
        //node* x_near = ptrTable[nearestNeighbor(x_rand)];
        node* x_near = ptrTable[nearestNeighbor(x_rand, MaxStep)];
        if (newState(out, x_near->location, x_rand, MaxStep)==0){
			point x_new;
			x_new.x = out[0];
			x_new.y = out[1];
			x_new.th = out[2];

			addVertex(x_new, x_rand, x_near->idx, out[3], out[4]);
		}
    }

    return 0;
}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    double y_rand = y_max + (rand() / ( RAND_MAX / (y_min-y_max))); 
    double x_rand = x_max + (rand() / ( RAND_MAX / (x_min-x_max)));
    point rand_point;
    rand_point.x = x_rand; 
    rand_point.y = y_rand;
    rand_point.th = NULL;

    return rand_point;

    //TODO
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    //TODO
	double max_theta = PI/2;//MaxStep * tan(max_alpha) / L;
    int x_near_index;
    double x_near_dist = 10000;
    double x_dist, theta;
    for(int i = 0; i < count; i++){
        if(ptrTable[i] != NULL){
            x_dist = sqrt(pow(ptrTable[i]->location.x - x_rand.x, 2) + pow(ptrTable[i]->location.y - x_rand.y, 2));

            if(x_dist < x_near_dist){
				theta = atan((ptrTable[i]->location.y - x_rand.y)/(ptrTable[i]->location.x - x_rand.x));
				if (theta < 0 && ptrTable[i]->location.x - x_rand.x > 0)
					theta += PI;
				else if (theta > 0 && ptrTable[i]->location.x - x_rand.x > 0)
					theta -= PI;

				/*
				printf("node: %f, %f, rand: %f, %f\n",ptrTable[i]->location.x, ptrTable[i]->location.y, x_rand.x, x_rand.y);
				printf("rate : %f\n", tan(theta));
				printf("theta: %f\n", theta);
				*/

				double error = theta - ptrTable[i]->location.th;
				if (error < -PI)
					error += 2*PI;
				else if (error > PI)
					error -= 2*PI;
                if (abs(error) < max_theta){
					x_near_index = i;
					x_near_dist = x_dist;
				}
            }
        }
    }

    return x_near_index;


}

int rrtTree::nearestNeighbor(point x_rand) {
    int x_near_index;
    double x_near_dist = 10000;
    double x_dist;
    for(int i = 0; i < count; i++){
        if(ptrTable[i] != NULL){
            x_dist = sqrt(pow(ptrTable[i]->location.x - x_rand.x, 2) + pow(ptrTable[i]->location.y - x_rand.y, 2));
            if(x_dist < x_near_dist){
                x_near_index = i;
                x_near_dist = x_dist;
            }
        }
    }

    return x_near_index;

    //TODO
}

int rrtTree::newState(double *out, point x_near, point x_rand, double MaxStep) {
    //TODO
    int num_paths = 1000;
    double alpha, d, R;
    double max_dist = 10000;
    double new_x, new_y, new_theta, new_d, new_R, new_alpha;
    int collision = 0;

    double x_dist = sqrt(pow(x_near.x - x_rand.x, 2) + pow(x_near.y - x_rand.y, 2));
	double max_theta = MaxStep * tan(max_alpha) / L;
	double theta = atan((x_near.y - x_rand.y)/(x_near.x - x_rand.x));
	if (theta < 0 && x_near.x - x_rand.x > 0)
		theta += PI;
	else if (theta > 0 && x_near.x - x_rand.x > 0)
		theta -= PI;
	double error = theta - x_near.th;
	if (error < -PI)
		error += 2*PI;
	else if (error > PI)
		error -= 2*PI;
	if (x_dist < MaxStep && abs(error) < max_theta){

		if (isCollision(x_near, x_rand, d, R))
			return 1;

		out[0] = x_rand.x;//new_x;
		out[1] = x_rand.y;//new_y;
		double gamma;
		if (error > 0)
			gamma = PI/2 - error;
		else
			gamma = -(PI/2 + error);
		R = x_dist / (2 * cos(gamma));	
		double beta = PI - 2 * abs(gamma);
		d = R * (PI - beta);
		alpha = atan(L * beta / d);
		if (gamma < 0){
			alpha = -alpha;
			beta = -beta;
		}
		out[2] = x_near.th + beta;
		out[3] = alpha;
		out[4] = d;
		return 0;
	}
    // generate some random paths and calc distance between new point and random point
    // save coordinates of closest point to random point
    for(int i=0;i<num_paths;i++){
        //alpha = rand() % max_alpha + max_alpha*-1;
        //d = rand() % MaxStep + 0;
        alpha = max_alpha + (rand() / ( RAND_MAX / (max_alpha*-1 - max_alpha)));
        d = MaxStep + (rand() / (RAND_MAX / (MaxStep)));
        R = L / tan(alpha);

		/*
        double x_c = x_near.x - R * sin(x_near.th);
        double y_c = x_near.y + R * sin(x_near.th);
		*/
        double beta = d / R;
		/*
        double x_bar = x_c + R * sin(x_near.th + beta);
        double y_bar = y_c - R * cos(x_near.th + beta);
		*/

        double x_bar = x_near.x + 2* R * sin(beta/2) * cos(x_near.th + beta/2);
        double y_bar = x_near.y - 2* R * sin(beta/2) * sin(x_near.th + beta/2);

        double x_dist = sqrt(pow(x_bar - x_rand.x, 2) + pow(y_bar - x_rand.y, 2));


        if(x_dist < max_dist){
            point x_new;
            x_new.x = x_bar;
            x_new.y = y_bar;

            if(!isCollision(x_near, x_new, d, R)){
                new_x = x_bar;
                new_y = y_bar;
                new_d = d;
                new_R = R;
                new_alpha = alpha;
                max_dist = x_dist;                
            }
        }
    }

	if (alpha > 0)
		new_theta = x_near.th + (new_d / new_R);
	else
		new_theta = x_near.th - (new_d / new_R);
	if (new_theta > PI)
		new_theta -= 2*PI;
	else if (new_theta < -PI)
		new_theta += 2*PI;
    out[0] = new_x;
    out[1] = new_y;
    out[2] = new_theta;
    out[3] = new_alpha;
    out[4] = new_d;


    return 0;
}

bool rrtTree::isCollision(point x1, point x2, double d, double R) {

    double i, j;
	int num_step = 500;
    // check for i points on the curve from x1 to x2 for collision
    for(int k = num_step; k >= 1; k--){
        double x_bar = (k * x1.x + (num_step-k) * x2.x)/num_step; 
        double y_bar = (k * x1.y + (num_step-k) * x2.y)/num_step; 

        i = x_bar / res + map_origin_x;
        j = y_bar / res + map_origin_y;

        if(map.at<uchar>(i,j) == 0 || map.at<uchar>(i,j) == 125){
            //std::cout << "Found collision";
            return true;
        }
    }

    return false;
    
    //TODO
}

std::vector<traj> rrtTree::backtracking_traj(){
    //TODO
    int near_index = nearestNeighbor(x_goal);
    node* goal_near = ptrTable[near_index];
	//near_index = nearestNeighbor(goal_near->location);
    node* current_node = ptrTable[goal_near->idx_parent];
    //node* current_node = goal_near;
    std::vector<traj> reversed_path;

	traj goal_traj;
	goal_traj.x = x_goal.x;
	goal_traj.y = x_goal.y;
	double x_ = current_node->location.x;
	double y_ = current_node->location.y;
	double th_ = atan((y_ - x_goal.y)/(x_ - x_goal.x));
	if (th_ < 0 && x_ - x_goal.x > 0)
		th_ += PI;
	else if (th_ > 0 && x_ - x_goal.x > 0)
		th_ -= PI;
	goal_traj.th = -th_;

	reversed_path.push_back(goal_traj);

    while(current_node->idx != 0){
		node* next_node = ptrTable[current_node->idx_parent];
		double theta = atan((next_node->location.y-current_node->location.y)/(next_node->location.x-current_node->location.x));

        traj next_traj;
        next_traj.x = current_node->location.x;
        next_traj.y = current_node->location.y;
        next_traj.th = theta; 
        next_traj.d = current_node->d;
        next_traj.alpha = current_node->alpha;

        reversed_path.push_back(next_traj);

        current_node = ptrTable[current_node->idx_parent];
        //std::cout << "x: " << next_traj.x << "y: " << next_traj.y << "\n";
    }
    //std::cout << "found path to closest point to goal!";

    //visualizeTree();

    return reversed_path;
}


