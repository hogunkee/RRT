#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846

double max_alpha = 0.30;//0.2;
double L = 0.325;

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
	weight[0] = 0;
}

rrtTree::rrtTree(point x_init, point x_goal) {
    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
	weight[0] = 0;
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
		weight[i] = 1000;
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
	weight[0] = 0;
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
	//printf("x: %f, y: %f\n", x_new.x, x_new.y);
	node* new_node = new node;
	new_node->idx = count;
	new_node->idx_parent = idx_near;
	new_node->location = x_new;
	new_node->rand = x_rand;
	new_node->alpha = alpha;
	new_node->d = d;
	ptrTable[count] = new_node;
	weight[count++] = weight[idx_near] + d;
	//printf("dist: %.2f", d);
	//std::cout << idx_near << " // " << weight[idx_near] << std::endl;	//plzdel
	/*
	printf("random point -");
	printPoint(x_rand);
	printf("\nnear point -");
	printPoint(ptrTable[idx_near]->location);
	printf("\nnew point -");
	printPoint(x_new);
	printf("\n\n");
	*/
}

void rrtTree::printPoint(point p){
	printf("x: %.2f,  y: %.2f,  th: %.2f", p.x, p.y, p.th);
}

int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
	//printf("generate RRT\n");
	double out[5];
	int iter_count=1;
	int check=0;
	while(true){
		if ((iter_count > K && check >=30) || iter_count++ > 10000 || count > 20000)// && check >= 1 && iter_count < 30000)
			break;

		point x_rand = randomState(x_max, x_min, y_max, y_min);

		if (iter_count % 30 == 0)
			x_rand = x_goal;
		if (iter_count % 65 == 1){
			point x_tmp;
			x_tmp.x = (x_goal.x + x_init.x)/2;
			x_tmp.y = (x_goal.y + x_init.y)/2;
			x_rand = x_tmp;
		}

		node* near_node;
		int near_idx = nearestNeighbor(x_rand, MaxStep);
		int second_idx = secondNearestNeighbor(x_rand, MaxStep);
		if (near_idx != -1)
			near_node = ptrTable[near_idx];
			//near_node = ptrTable[nearestNeighbor(x_rand, MaxStep)];
		if (near_idx != -1 && newState(out, near_node->location, x_rand, MaxStep)==0){
			point x_new;
			x_new.x = out[0];
			x_new.y = out[1];
			x_new.th = out[2];

			/* arrive at the goal point */
			double x_dist = sqrt(pow(x_goal.x - x_new.x, 2) + pow(x_goal.y - x_new.y, 2));
			if (x_dist < 2){
				check++;
			}
			node* tmp = near_node;
			addVertex(x_new, x_rand, tmp->idx, out[3], out[4]);
			/* RRT-STAR */
			if (tmp->idx_parent!=NULL && tmp->idx_parent!=0){
				tmp = ptrTable[tmp->idx_parent];
				double dist = sqrt(pow(tmp->location.x - x_new.x, 2) + pow(tmp->location.y - x_new.y, 2));
				if(dist < 2.5*MaxStep && newState(out, tmp->location, x_new, 100)==0){
				//if(newState(out, ptrTable[tmp->idx_parent]->location, x_new, 5*MaxStep)==0){
					addVertex(x_new, x_rand, tmp->idx, out[3], out[4]);
				}
			}
			
		}
		if (second_idx != -1)
			near_node = ptrTable[second_idx];
		if (second_idx != -1 && newState(out, near_node->location, x_rand, MaxStep)==0){
			point x_new;
			x_new.x = out[0];
			x_new.y = out[1];
			x_new.th = out[2];

			addVertex(x_new, x_rand, second_idx, out[3], out[4]);
		}
	}
	return 0;
}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
//	printf("random state\n");//
    double y_rand = y_max + (rand() / ( RAND_MAX / (y_min-y_max))); 
    double x_rand = x_max + (rand() / ( RAND_MAX / (x_min-x_max)));
	point rand_point;
	rand_point.x = x_rand;
	rand_point.y = y_rand;
	return rand_point;
}

int rrtTree::secondNearestNeighbor(point x_rand, double MaxStep) {
	double th_g, R, alpha, error;
	int x_near_index = -1;
	int second_index = -1;
	double x_near_dist = 10000;
	double x_dist;

	for (int i=0; i<count; i++){
		if (ptrTable[i] != NULL){
			point tmp = ptrTable[i]->location;
            x_dist = sqrt(pow(tmp.x - x_rand.x, 2) + pow(tmp.y - x_rand.y, 2));
			
			if (tmp.x==x_rand.x && tmp.y==x_rand.y)
				th_g = tmp.th;
			else
				th_g = atan((tmp.y - x_rand.y)/(tmp.x - x_rand.x));

			if (th_g <= 0 && tmp.x - x_rand.x > 0)
				th_g += PI;
			else if (th_g > 0 && tmp.x - x_rand.x > 0)
				th_g -= PI;

			error = th_g - tmp.th;
			if (error < -PI)
				error += 2*PI;
			else if (error > PI)
				error -= 2*PI;

			if (error < PI/2 && error > -PI/2){
				alpha = atan(2*L*sin(error)/x_dist);
				if(x_dist <= x_near_dist && alpha < max_alpha){
					second_index = x_near_index;
					x_near_index = i;
					x_near_dist = x_dist;
				}
			}
		}
	}
	return second_index;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
	double th_g, R, alpha, error;
	int x_near_index = -1;
	double x_near_dist = 10000;
	double x_dist;
	double x_near_weight = 1000;
	for (int i=0; i<count; i++){
		if (ptrTable[i] != NULL){
			point tmp = ptrTable[i]->location;

            x_dist = sqrt(pow(tmp.x - x_rand.x, 2) + pow(tmp.y - x_rand.y, 2));
			double w = weight[i] + x_dist;
			
			if (tmp.x==x_rand.x && tmp.y==x_rand.y)
				th_g = tmp.th;
			else
				th_g = atan((tmp.y - x_rand.y)/(tmp.x - x_rand.x));

			if (th_g <= 0 && tmp.x - x_rand.x > 0)
				th_g += PI;
			else if (th_g > 0 && tmp.x - x_rand.x > 0)
				th_g -= PI;

			error = th_g - tmp.th;
			if (error < -PI)
				error += 2*PI;
			else if (error > PI)
				error -= 2*PI;

			if (error < PI/2 && error > -PI/2){
				alpha = atan(2*L*sin(error)/x_dist);

				/*
				if(x_dist <= x_near_dist && alpha < max_alpha){
					x_near_index = i;
					x_near_dist = x_dist;
					x_near_weight = w;
					if (x_near_dist < MaxStep)
						return x_near_index;
				}
				*/
				if(x_dist <= 2*MaxStep && alpha < max_alpha && w > 0 && w < x_near_weight){
					x_near_index = i;
					x_near_dist = x_dist;
					x_near_weight = w;
					if (x_near_dist < MaxStep)
						return x_near_index;
				}
				else if(x_dist <= x_near_dist && alpha < max_alpha){
					x_near_index = i;
					x_near_dist = x_dist;
					x_near_weight = w;
				}
			}
		}
	}
	if (x_near_index == -1)
		return nearestNeighbor(x_rand);
	return x_near_index;
}

int rrtTree::nearestNeighbor(point x_rand) {
	//printf("nearest neighbor\n");//
	int x_near_index;
	double x_near_dist = 10000;
	double x_dist;

	for (int i=0; i<count; i++){
		if (ptrTable[i] != NULL){
			point tmp = ptrTable[i]->location;
            x_dist = sqrt(pow(tmp.x - x_rand.x, 2) + pow(tmp.y - x_rand.y, 2));

            if(x_dist < x_near_dist){
                x_near_index = i;
                x_near_dist = x_dist;
			}
		}
	}
	return x_near_index;
}

int rrtTree::newState(double *out, point x_near, point x_rand, double MaxStep) {
	//printf("new state\n");//
	double x_dist, th_g, error, D, R, alpha, beta;

	x_dist = sqrt(pow(x_near.x - x_rand.x, 2) + pow(x_near.y - x_rand.y, 2));
	
	if (x_near.x==x_rand.x && x_near.y==x_rand.y)
		th_g = x_near.th;
	else
		th_g = atan((x_near.y - x_rand.y)/(x_near.x - x_rand.x));

	if (th_g < 0 && x_near.x - x_rand.x > 0)
		th_g += PI;
	else if (th_g > 0 && x_near.x - x_rand.x > 0)
		th_g -= PI;

	error = th_g - x_near.th;
	if (error < -PI)
		error += 2*PI;
	else if (error > PI)
		error -= 2*PI;

	R = x_dist / (2 * sin(error));
	D = x_dist * (error / sin(error));
	if (abs(error) > PI/2)
		alpha = (error > 0) ? 2 * max_alpha: -2 * max_alpha;
	else
		alpha = atan(2*L*sin(error)/x_dist);

	/* feasible and distance is less then MaxStep */
	if (D < MaxStep && abs(alpha) < max_alpha){
		if (isCollision(x_near, x_rand, D, R))
			return 1;

		beta = D/R;
		out[0] = x_rand.x;
		out[1] = x_rand.y;
		out[2] = x_near.th + beta;
		if (out[2] > PI)
			out[2] -= 2*PI;
		else if (out[2] < -PI)
			out[2] += 2*PI;
		out[3] = alpha;
		out[4] = D;

		return 0;
	}

	/* out of range */
	if (alpha > max_alpha || alpha < -max_alpha)
		alpha = (alpha > 0) ? max_alpha : -max_alpha;

	double d = (MaxStep + (rand() / (RAND_MAX / (MaxStep))))/2;
	R = L / tan(alpha);
	beta = d*tan(alpha)/L;

	double x_, y_;
	x_ = x_near.x + (sin(x_near.th + beta) - sin(x_near.th)) * L / tan(alpha);
	y_ = x_near.y - (cos(x_near.th + beta) - cos(x_near.th)) * L / tan(alpha);

	point x_new;
	x_new.x = x_;
	x_new.y = y_;
	if (isCollision(x_near, x_new, d, R))
		return 1;

	out[0] = x_;
	out[1] = y_;
	out[2] = x_near.th + beta;
	if (out[2] > PI)
		out[2] -= 2*PI;
	else if (out[2] < -PI)
		out[2] += 2*PI;
	out[3] = alpha;
	out[4] = d;

	return 0;
}

bool rrtTree::isCollision(point x1, point x2, double d, double R) {
	/*
	//printf("collision\n");//
    double i, j;
	int num_step = 500;
    // check for i points on the curve from x1 to x2 for collision
    for(int k = num_step; k >= 1; k--){
        double x_bar = (k * x1.x + (num_step-k) * x2.x)/num_step; 
        double y_bar = (k * x1.y + (num_step-k) * x2.y)/num_step; 

        i = x_bar / res + map_origin_x;
        j = y_bar / res + map_origin_y;

        if(map.at<uchar>(i,j) == 0 || map.at<uchar>(i,j) == 125){
            return true;
        }
    }

    return false;
	/**/
	int num_step1 = 40;//500;
	int num_step2 = 100;//500;
	double x_dist, th_g, error, alpha, beta;
	//double D, R;

	x_dist = sqrt(pow(x1.x - x2.x, 2) + pow(x1.y - x2.y, 2));
	if (x1.x==x2.x && x1.y==x2.y)
		th_g = x1.th;
	else
		th_g = atan((x1.y - x2.y)/(x1.x - x2.x));

	if (th_g < 0 && x1.x - x2.x > 0)
		th_g += PI;
	else if (th_g > 0 && x1.x - x2.x > 0)
		th_g -= PI;

	error = th_g - x1.th;
	if (error < -PI)
		error += 2*PI;
	else if (error > PI)
		error -= 2*PI;

	//R = x_dist / (2 * sin(error));
	//D = 2 * R * cos(PI/2 - abs(error));
	//alpha = (error > 0) ? atan(L/R) : -atan(L/R);
	//beta = (alpha > 0) ? D/R : -D/R;

	alpha = atan(2*L*sin(error)/x_dist);
	if (alpha > max_alpha || alpha < -max_alpha)
		return true;
	beta = 2*error;
	//alpha = (error > 0) ? atan(L/R) : -atan(L/R);
	//beta = (alpha > 0) ? d/R : -d/R;

	double x_, y_, pre_x_, pre_y_, b_;
	double i, j;
	pre_x_ = x1.x / res;
	pre_y_ = x1.y / res;
	for (int k=0; k<=num_step1; k++){
		b_ = k / num_step1 * beta;
		x_ = x1.x / res + (sin(x1.th + b_)/ res /tan(alpha) - sin(x1.th)/ res /tan(alpha)) * L;
		y_ = x1.y / res - (cos(x1.th + b_)/ res /tan(alpha) - cos(x1.th)/ res /tan(alpha)) * L;
		//printf("%.3f\t%.3f\n\n",x_,y_);
		//x_ = x1.x + 2*R*sin(b_/2)*cos(x1.th + b_/2);
		//y_ = x1.y + 2*R*sin(b_/2)*sin(x1.th + b_/2);

		for(int l = num_step2; l >= 1; l--){
			double x_bar = (l * x_ + (num_step2-l) * pre_x_)/num_step2; 
			double y_bar = (l * y_ + (num_step2-l) * pre_y_)/num_step2; 

			i = x_bar + map_origin_x;
			j = y_bar + map_origin_y;

			if(map.at<uchar>(i,j) == 0 || map.at<uchar>(i,j) == 125){
				return true;
			}
        }
		pre_x_ = x_;
		pre_y_ = y_;
	}
	return false;
	//*/
}

std::vector<traj> rrtTree::backtracking_traj(){
	//printf("backtrack\n");//

	point x_rand = x_goal;
	double out[5];
	int checkBack = 0;
	/* back one step from goal point */
	//if (x_goal.th != 0.00 && x_goal.th != NULL){
	if (abs(x_goal.th) > 0.01){ 
		point tmp = x_goal;
		tmp.th = x_goal.th + PI;
		if (tmp.th > PI)
			tmp.th -= 2*PI;
		if (newState(out, tmp, x_init, 0.5)==0){
			/*
			printf("----------New Policy!!---------\n");
			printf("goal theta: %.3f\n", x_goal.th);
			*/
			checkBack = 1;
			x_rand.x = out[0];
			x_rand.y = out[1];
			x_rand.th = out[2] + PI;
			if (x_rand.th > PI)
				x_rand.th -= 2*PI;
		}
	}

	int near_index = nearestNeighbor(x_rand, 1);

    node* goal_near = ptrTable[near_index];
    node* current_node = ptrTable[goal_near->idx_parent];
    std::vector<traj> reversed_path;

	traj goal_traj;
	double x_, y_, th_;
	if (checkBack == 0) {
		goal_traj.x = x_goal.x;
		goal_traj.y = x_goal.y;
		x_ = current_node->location.x;
		y_ = current_node->location.y;
		th_ = (x_ - x_goal.x != 0) ? atan((y_ - x_goal.y)/(x_ - x_goal.x)) : current_node->location.th;
		if (th_ < 0 && x_ - x_goal.x > 0)
			th_ += PI;
		else if (th_ > 0 && x_ - x_goal.x > 0)
			th_ -= PI;
		goal_traj.th = th_;

		reversed_path.push_back(goal_traj);
	} else {
		goal_traj.x = x_goal.x;
		goal_traj.y = x_goal.y;
		/*
		x_ = x_rand.x;
		y_ = x_rand.y;
		th_ = (x_ - x_goal.x != 0) ? atan((y_ - x_goal.y)/(x_ - x_goal.x)) : x_rand.th;
		if (th_ < 0 && x_ - x_goal.x > 0)
			th_ += PI;
		else if (th_ > 0 && x_ - x_goal.x > 0)
			th_ -= PI;
		*/
		goal_traj.th = x_goal.th;
		reversed_path.push_back(goal_traj);

		goal_traj.x = x_rand.x;
		goal_traj.y = x_rand.y;
		goal_traj.th = x_rand.th;
		reversed_path.push_back(goal_traj);
	}

    while(current_node->idx != 0){
		node* next_node = ptrTable[current_node->idx_parent];
		//int w = weight[current_node->idx_parent];

        traj next_traj;
        next_traj.x = current_node->location.x;
        next_traj.y = current_node->location.y;
        next_traj.th = current_node->location.th;//theta; 
        next_traj.d = current_node->d;
        next_traj.alpha = current_node->alpha;

        reversed_path.push_back(next_traj);

        current_node = ptrTable[current_node->idx_parent];
    }
    //visualizeTree();

    return reversed_path;
}
