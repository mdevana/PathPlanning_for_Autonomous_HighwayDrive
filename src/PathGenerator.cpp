#include "PathGenerator.h"
#include <vector>
#include <math.h>
//#include "helpers_planning.h"
#include <string>
#include <iostream>



using std::vector;
using std::string;
using std::vector;


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

void PathGenerator::set_previous_path_data(const vector<double> &x,const vector<double> &y, double prev_s, double prev_d) {
  
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
			//next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
			//next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
		}
}

void PathGenerator::generate_circular_path(){
	/*
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
	*/
		
}


void PathGenerator::generate_map_path(){
	
	lanecode followlane=middle;
	double pos_x;
	double pos_y;
	double angle;
	int path_size = previous_path_x.size();
	std::cout <<"Previous Path size :" <<path_size<< std::endl;

	for (int i = 0; i < path_size; ++i) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}


	
	std::cout << "dist increment = "<<dist_inc << std::endl;
	std::cout << "end S = "<<end_s << std::endl;
	WayPoint current_wp;
	
	std::cout << path_size<< std::endl;
	double dist_inc = 0.44 ;
	
	double prev_x_coor = car_x;
	double prev_y_coor = car_y;
	
	int cnt_start_path_pts = 0;
	
	if (path_size == 0){
		
		
		dist_inc = 0.44 ;
		double angle = (car_yaw) * M_PI / 180;
		double end_x_coor;
		double end_y_coor;
		
		end_s = 0;
		
				
		while (end_s < 121) {    
			
			
			
			end_x_coor = car_x+(dist_inc * cnt_start_path_pts)*cos(angle);
			end_y_coor = car_y+(dist_inc * cnt_start_path_pts)*sin(angle);
			
			std::cout <<"Car X =" <<end_x_coor << std::endl;
			std::cout <<"Car y =" <<end_y_coor << std::endl;
			
			next_x_vals.push_back(end_x_coor);
			next_y_vals.push_back(end_y_coor);
			cnt_start_path_pts++;
			
			end_s = (highway_map.get_map_convertedS_for_XY(end_x_coor,end_y_coor,angle)).get_s_co();
			std::cout <<"End S  =" <<end_s << std::endl;
			std::cout <<"distance to previous point  =" <<sqrt((end_x_coor-prev_x_coor)*(end_x_coor-prev_x_coor)+(end_y_coor-prev_y_coor)*(end_y_coor-prev_y_coor))<< std::endl;
			
			prev_x_coor = end_x_coor;
			prev_y_coor = end_y_coor;
		}

		// S needs to be above 121 to merge into the path
	}
	
	
	
	
	for (int j = 0; j < (50-path_size-cnt_start_path_pts); ++j) {    
	    
		double new_s = end_s + dist_inc * j ;
		std::cout << "new S = "<<new_s << std::endl;
		
		current_wp = highway_map.get_map_convertedXY_for_s(new_s,followlane);
		//waypoint pt = way_pts[i];
		
		double current_x = current_wp.get_x_co();
		double current_y = current_wp.get_y_co();
		
		std::cout <<"Current S =" <<current_wp.get_s_co() << std::endl;
		
		std::cout <<"Current X =" <<current_x << std::endl;
		std::cout <<"Current Y =" <<current_y<< std::endl;
		
		std::cout <<"distance to previous point  =" <<sqrt((current_x-prev_x_coor)*(current_x-prev_x_coor)+(current_y-prev_y_coor)*(current_y-prev_y_coor))<< std::endl;
		
		
		next_x_vals.push_back(current_wp.get_x_co());
		next_y_vals.push_back(current_wp.get_y_co());
		
		prev_x_coor = current_x;
		prev_y_coor = current_y;
		
		
	}
	
}

void PathGenerator::generate_map_path_JMT(){
	
	lanecode followlane=left;
	double pos_x;
	double pos_y;
	double angle;
	int path_size = previous_path_x.size();
	std::cout <<"Previous Path size :" <<path_size<< std::endl;

	for (int i = 0; i < path_size; ++i) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	WayPoint current_wp;
	
	std::cout << path_size<< std::endl;
	double dist_inc = 0.44 ;
	
	std::cout << "dist increment = "<<dist_inc << std::endl;
	std::cout << "end S = "<<end_s << std::endl;
	
	int cnt_start_path_pts = 1;
	
	if (path_size == 0){
		
		
		dist_inc = 0.44 ;
		double angle = (car_yaw) * M_PI / 180;
		double end_x_coor;
		double end_y_coor;
		
		double prev_x_coor = 0;
		double prev_y_coor = 0;
		
		end_s = 0;
		end_d = 0;
		
		while (end_s < 121) {    
			
			prev_x_coor = car_x;
			prev_y_coor = car_y;
			
			end_x_coor = car_x+(dist_inc * cnt_start_path_pts)*cos(angle);
			end_y_coor = car_y+(dist_inc * cnt_start_path_pts)*sin(angle);
			
			std::cout <<"Car X =" <<end_x_coor << std::endl;
			std::cout <<"Car y =" <<end_y_coor << std::endl;
			
			next_x_vals.push_back(end_x_coor);
			next_y_vals.push_back(end_y_coor);
			cnt_start_path_pts++;
			
			WayPoint wp = highway_map.get_map_convertedS_for_XY(end_x_coor,end_y_coor,angle);
			
			end_s = wp.get_s_co();
			end_d = wp.get_d_co();
			
			std::cout <<"End S  =" <<end_s << std::endl;
			std::cout <<"distance to previous point  =" <<sqrt((end_x_coor-prev_x_coor)*(end_x_coor-prev_x_coor)+(end_y_coor-prev_y_coor)*(end_y_coor-prev_y_coor))<< std::endl;
		}
		
		
		// S needs to be above 121 to merge into the path
	}
	
 
	    
		double final_s = end_s + dist_inc * (50-path_size-cnt_start_path_pts);
		
		std::cout <<"Distance to Predict to next set: " <<dist_inc * (50-path_size-cnt_start_path_pts)<< std::endl;
		std::cout <<"Predict to next path: Final S =" <<final_s<< std::endl;
		
		double final_d = end_d;
		
		vector<double> s_start{end_s,max_velocity,10};
		vector<double> s_end{final_s,max_velocity,10};
		
		vector<double> d_start{end_d,max_velocity,10};
		vector<double> d_end{final_d,max_velocity,10};
		
		double start_time=0;
		double end_time= (final_s - end_s) / max_velocity;
		double time_inc = dist_inc/max_velocity;
		
		std::cout <<"End time: " <<end_time<< std::endl;
		std::cout <<"time increments =" <<time_inc<< std::endl;

		vector<WayPoint> current_wp_points = highway_map.get_map_convertedSD_for_XY_jerk_optimised(s_start, s_end, d_start, d_end, start_time, end_time, time_inc, followlane);
		//vector<double> &s_start,vector<double> &s_end, vector<double> &d_start, vector<double> &d_end, double start_time, double start_time, double end_time, double inc
		
		
		for(WayPoint wp : current_wp_points){
			
			std::cout <<"Current S =" <<wp.get_s_co() << std::endl;
		
			std::cout <<"Current X =" <<wp.get_x_co() << std::endl;
			std::cout <<"Current Y =" <<wp.get_y_co() << std::endl;
		
		
			next_x_vals.push_back(wp.get_x_co());
			next_y_vals.push_back(wp.get_y_co());
		}

	
}



