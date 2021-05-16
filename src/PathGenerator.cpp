#include "PathGenerator.h"
#include <vector>
#include <math.h>
#include "helpers_planning.h"

using std::vector;

PathGenerator::PathGenerator() {}

PathGenerator::~PathGenerator() {}

void PathGenerator::Init(double inc) {
  
  dist_inc = inc;

}

void PathGenerator::set_localization_data(double x,double y, double yaw) {
  
  car_x = x;
  car_y = y;
  car_yaw = yaw;

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
