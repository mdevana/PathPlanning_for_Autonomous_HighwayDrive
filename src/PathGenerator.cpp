#include "PathGenerator.h"
#include <vector>
#include <math.h>
#include "helpers_planning.h"
#include <string>
#include "MapPath.h"
#include "Dense"



using std::vector;
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

PathGenerator::PathGenerator() {}

PathGenerator::~PathGenerator() {}

void PathGenerator::Init(double inc, MapPath mp) {
  
  dist_inc = inc;
  highway_map=mp;
  

}

void PathGenerator::set_localization_data(double x,double y, double s, double d,double yaw,double speed) {
  
  car_x = x;
  car_y = y;
  car_s = s;
  car_d = d;
  car_yaw = yaw;
  car_speed = speed;

}

void PathGenerator::set_previous_path_data(vector<double> x,vector<double> y, double prev_s, double prev_d) {
  
  previous_path_x = x;
  previous_path_y = y;
  end_s = prev_s; 
  end_d = prev_d;

}



vector<double> PathGenerator::get_x_vals() {
  
  return next_x_vals;  
}

vector<double> PathGenerator::get_y_vals() {
  
  return next_y_vals;  
}

void PathGenerator::generate_simple_path(){
	
		for (int i = 0; i < 50; ++i) {
			next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
			next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
		}
}

void PathGenerator::generate_circular_path(){
	
	double pos_x;
	double pos_y;
	double angle;
	int path_size = previous_path_x.size();

	for (int i = 0; i < path_size; ++i) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	if (path_size == 0) {
		pos_x = car_x;
		pos_y = car_y;
		angle = deg2rad(car_yaw);
	} else {
		pos_x = previous_path_x[path_size-1];
		pos_y = previous_path_y[path_size-1];

		double pos_x2 = previous_path_x[path_size-2];
		double pos_y2 = previous_path_y[path_size-2];
		angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
	}

	//double dist_inc = 0.5;
	for (int i = 0; i < 50-path_size; ++i) {    
		next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
		next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
		pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
		pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
	}

		
}


void PathGenerator::generate_map_path(){
	
	double pos_x;
	double pos_y;
	double angle;
	int path_size = previous_path_x.size();

	for (int i = 0; i < path_size; ++i) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	if (path_size == 0) {
		pos_x = car_x;
		pos_y = car_y;
		angle = deg2rad(car_yaw);
	} else {
		pos_x = car_x;
		pos_y = car_y;
		angle = deg2rad(car_yaw);
		
	}

	//double dist_inc = 0.5;
	
	
	for (int i = 0; i < 50-path_size; ++i) {    
	    
		//waypoint pt = way_pts[i];
		
		//next_x_vals.push_back(pt.x_co);
		//next_y_vals.push_back(pt.y_co);
		
		//pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
		//pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
	}
	
}

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
   MatrixXd TimeMat(3,3);
   TimeMat <<pow(T,3.0), pow(T,4.0), pow(T,5),
             3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
             6*T, 12*pow(T,2), 20*pow(T,3);
    
    MatrixXd TimeMat_inv(3,3);
    TimeMat_inv = TimeMat.inverse();
    
    VectorXd initial_C(3);
    initial_C<<end[0] - ( start[0] + start[1] * T + 0.5 * start[2] * pow(T,2) ),
                end[1] -( start[1] + start[2] * T),
                end[2] - start[2];
    VectorXd Coeff_456(3);
    Coeff_456 = TimeMat_inv * initial_C;
    
  //vector<double> coeff{start[0],start[1],0.5 * start[2],Coeff_456[3],Coeff_456[4],Coeff_456[5]};
  return {start[0], start[1], 0.5 * start[2], Coeff_456[0], Coeff_456[1], Coeff_456[2]};
}

