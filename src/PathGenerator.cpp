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
			next_x_vals.push_back(car_x+(dist_inc*i)*cos(car_yaw * M_PI / 180));
			next_y_vals.push_back(car_y+(dist_inc*i)*sin(car_yaw * M_PI / 180));
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
	
	int path_size = previous_path_x.size();
	int i;
	
	std::cout<< " size of path "<<path_size;
	
	
}

void PathGenerator::Execute_lane_change_with_JMT(){
	
	
		double final_s = end_s + dist_inc * (200);
		
		std::cout <<"Distance to Predict to next set: " <<dist_inc * (200)<< std::endl;
		std::cout <<"Predict to next path: Final S =" <<final_s<< std::endl;
		
		double final_d = 2;
		
		vector<double> s_start{end_s,car_speed,0};
		vector<double> s_end{final_s,max_velocity,0};
		
		vector<double> d_start{6,0,0};
		vector<double> d_end{2,0,0};
		
		double start_time=0;
		double end_time= (final_s - end_s) / max_velocity;
		double time_inc = dist_inc/max_velocity;
		
		std::cout <<"End time: " <<end_time<< std::endl;
		std::cout <<"time increments =" <<time_inc<< std::endl;

		vector<WayPoint> current_wp_points = highway_map.get_map_convertedSD_for_XY_jerk_optimised(s_start, s_end, d_start, d_end, start_time, end_time, time_inc);
		//vector<double> &s_start,vector<double> &s_end, vector<double> &d_start, vector<double> &d_end, double start_time, double start_time, double end_time, double inc
		
		
		for(WayPoint wp : current_wp_points){
			
			double current_x = wp.get_x_co();
		    double current_y = wp.get_y_co();
			
			std::cout <<"Current S =" <<wp.get_s_co() << std::endl;
		
			std::cout <<"Current X =" <<wp.get_x_co() << std::endl;
			std::cout <<"Current Y =" <<wp.get_y_co() << std::endl;
			
			//std::cout <<"distance to previous point  =" <<sqrt((current_x-prev_x_coor)*(current_x-prev_x_coor)+(current_y-prev_y_coor)*(current_y-prev_y_coor))<< std::endl;		
		
			next_x_vals.push_back(wp.get_x_co());
			next_y_vals.push_back(wp.get_y_co());
			
			//prev_x_coor = current_x;
		    //prev_y_coor = current_y;
		}

	
}

void PathGenerator::generate_map_path_with_traffic(vector<vector<double>> sensor_fusion){
	
	
	int path_size = previous_path_x.size();
	std::cout<< " size of path "<<path_size;
	
	
	// step 1 : copy unexecuted Path
	copy_unexecuted_path(); 

	// step 2 : initialise Ego Vehicle
	//int id, double x, double y,double s, double d, double v, double yaw, string state
	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = (car_yaw) * M_PI / 180;
	double ref_velocity = car_speed;
		
	Vehicle ego_vehicle(1000,car_x,car_y,car_s,car_d,car_speed,car_yaw,"CS");
	
	// step 3 : Make traffic Predictions
	make_traffic_predictions(sensor_fusion); 
	
	
	if (path_size ==0){
		end_s = car_s;

	/*	double prev_x_coor = car_x;
		double prev_y_coor = car_y;
		
		end_s = 0;
		
		int cnt_start_path_pts=1;
				
		while (end_s < 125) {    
			
			
			vector<double> end_coor = ego_vehicle.cold_start(simulator_time_step);
			//double end_x_coor = car_x+(0.02 * max_velocity * cnt_start_path_pts)*cos(ref_yaw);
			//double end_y_coor = car_y+(0.02 * max_velocity * cnt_start_path_pts)*sin(ref_yaw);
			
			std::cout <<"Car X =" <<end_coor[0]<< std::endl;
			std::cout <<"Car y =" <<end_coor[1] << std::endl;
			
			next_x_vals.push_back(end_coor[0]);
			next_y_vals.push_back(end_coor[1]);
			cnt_start_path_pts++;
			
			end_s = (highway_map.get_map_convertedS_for_XY(end_coor[0],end_coor[1],ref_yaw)).get_s_co();
			std::cout <<"End S  =" <<end_s << std::endl;
			std::cout <<"distance to previous point  =" <<sqrt((end_coor[0]-prev_x_coor)*(end_coor[0]-prev_x_coor)+(end_coor[1]-prev_y_coor)*(end_coor[1]-prev_y_coor))<< std::endl;
			
			prev_x_coor = end_coor[0];
			prev_y_coor = end_coor[1];
		}*/

		// S needs to be above 121 to merge into the path

		
	}
	
	vector<double> pts_x;
	vector<double> pts_y;
	
	if ( path_size < 2){
		
		// 2 points that makes path tangent to the car
		pts_x.push_back(car_x - cos(car_yaw));
		pts_y.push_back(car_y - sin(car_yaw));
		
		pts_x.push_back(car_x);
		pts_y.push_back(car_y);
		
	}
	else {
		
		
		
		// Reference pt is two last point in the Q
		ref_x = previous_path_x[path_size -1];
		ref_y = previous_path_y[path_size -1];
		
		double ref_x_1 = previous_path_x[path_size -2];
		double ref_y_1 = previous_path_y[path_size -2];
		
		ref_yaw = atan2( ref_y - ref_y_1, ref_x - ref_x_1);
		
		pts_x.push_back(ref_x_1);
		pts_x.push_back(ref_x);
		
		pts_y.push_back(ref_y_1);
		pts_y.push_back(ref_y);

		
	}
	
	std::cout <<"End S  =" <<end_s << std::endl;
	highway_map.calculate_map_XYspline_for_s(end_s, 6, pts_x, pts_y,ref_yaw);
		
	
	
	
	double x_estimate = 30;
	double y_estimate = highway_map.get_y_from_curve(x_estimate);
	double dist_estimate = sqrt(x_estimate * x_estimate + y_estimate * y_estimate);
	
	
	
	if( car_speed < max_velocity ){
		
		ref_velocity = car_speed + 9 * simulator_time_step * (50 - path_size);

	}
	else 
		ref_velocity = max_velocity;
		
	std::cout <<"Ref velocity= " <<ref_velocity<< std::endl;
		
	double n_dist_inc = dist_estimate / (0.02*ref_velocity);
	double dist_inc_x = x_estimate / n_dist_inc;
	double x_pt = 0;
	double y_pt;
	
		for(int i = 1; i<= 50 - path_size;i++){
			
			x_pt += dist_inc_x;
			y_pt = highway_map.get_y_from_curve(x_pt);

			
			next_x_vals.push_back(ref_x + (x_pt * cos(ref_yaw)  - y_pt * sin(ref_yaw)));
			next_y_vals.push_back(ref_y + (x_pt * sin(ref_yaw)  + y_pt * cos(ref_yaw)));

			
		}
	
	
		
	
	
	

}

void PathGenerator::copy_unexecuted_path(){
	
	for(int i=0;i< previous_path_x.size();i++){
		
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}
	
}

void PathGenerator::make_traffic_predictions(vector<vector<double>> sensor_fusion){
	
	std::cout <<"Size =" <<sensor_fusion.size() << std::endl;
	
	int path_size = previous_path_x.size();
	
	for(int i = 0 ; i <sensor_fusion.size(); i++){
		//int id, double x, double y,double s, double d, double vx, double vy, string state
		int id = sensor_fusion[i][0];
		//std::cout<< " inserting vehicles on road id at : "<<i<<" : " <<id<<std::endl;
	    Vehicle v(id,sensor_fusion[i][1],sensor_fusion[i][2],sensor_fusion[i][3],sensor_fusion[i][4],sensor_fusion[i][5],sensor_fusion[i][6],"CS");
		//v.VehicleParamDisplay();
		v.generate_predictions(path_size, simulator_time_step);	
		vehicles_in_road.insert(std::pair<int,Vehicle>(id,v));
	}
	
	/*Debugging CODE
	map<int, Vehicle>::iterator it = vehicles_in_road.begin();
	Vehicle vh;
	
	while(it != vehicles_in_road.end()){
		int v_id = it->first;
		std::cout<< " reading out vehicles on road id : " << v_id<<std::endl;
		it->second.VehicleParamDisplay();
		++it;
	}*/
}

void PathGenerator::generate_map_path_with_transform(){
	
	
	lanecode followlane = middle;
	int lane_change = 0;
	
	int path_size = previous_path_x.size();
	int i;
	
	std::cout<< " size of path "<<path_size<<std::endl;
	
	//generate_simple_path();
	
	for(i=0;i< path_size;i++){
		
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}
	
	// collect previous 2 points
	vector<double> pts_x;
	vector<double> pts_y;
	
	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = (car_yaw) * M_PI / 180;
	
	std::cout<< " car yaw "<<ref_yaw<<std::endl;
	std::cout<< " car X "<<ref_x<<std::endl;
	std::cout<< " car Y "<<ref_y<<std::endl;
	
	double prev_x_coor = car_x;
	double prev_y_coor = car_y;
	
	
	if (path_size ==0){
		

		
		
		
		
		double end_x_coor;
		double end_y_coor;
		
		end_s = 0;
		
		int cnt_start_path_pts=1;
				
		while (end_s < 121) {    
			
			
			
			double end_x_coor = car_x+(0.02 * max_velocity * cnt_start_path_pts)*cos(ref_yaw);
			double end_y_coor = car_y+(0.02 * max_velocity * cnt_start_path_pts)*sin(ref_yaw);
			
			std::cout <<"Car X =" <<end_x_coor << std::endl;
			std::cout <<"Car y =" <<end_y_coor << std::endl;
			
			next_x_vals.push_back(end_x_coor);
			next_y_vals.push_back(end_y_coor);
			cnt_start_path_pts++;
			
			end_s = (highway_map.get_map_convertedS_for_XY(end_x_coor,end_y_coor,ref_yaw)).get_s_co();
			std::cout <<"End S  =" <<end_s << std::endl;
			//std::cout <<"distance to previous point  =" <<sqrt((end_x_coor-prev_x_coor)*(end_x_coor-prev_x_coor)+(end_y_coor-prev_y_coor)*(end_y_coor-prev_y_coor))<< std::endl;
			
			prev_x_coor = end_x_coor;
			prev_y_coor = end_y_coor;
		}

		// S needs to be above 121 to merge into the path

		
	}
	
	
	
	if ( path_size < 2){
		
		// 2 points that makes path tangent to the car
		pts_x.push_back(car_x - cos(car_yaw));
		pts_y.push_back(car_y - sin(car_yaw));
		
		pts_x.push_back(car_x);
		pts_y.push_back(car_y);
		
	}
	else {
		
		if (end_s > 500) {
			Execute_lane_change_with_JMT();
		}
		
		// Reference pt is two last point in the Q
		ref_x = previous_path_x[path_size -1];
		ref_y = previous_path_y[path_size -1];
		
		double ref_x_1 = previous_path_x[path_size -2];
		double ref_y_1 = previous_path_y[path_size -2];
		
		ref_yaw = atan2( ref_y - ref_y_1, ref_x - ref_x_1);
		
		pts_x.push_back(ref_x_1);
		pts_x.push_back(ref_x);
		
		pts_y.push_back(ref_y_1);
		pts_y.push_back(ref_y);

		
	}
	
	for(int j=0; j< pts_x.size(); j++){
		 std::cout<< " pts_x ["<<j<<"] = "<<pts_x[j]<<std::endl;
		 std::cout<< " pts_y ["<<j<<"] = "<<pts_y[j]<<std::endl;
		 //std::cout<< " pts_y ["<<j<<"] = "<<pts_x[0];
		 
	 }
	
	std::cout <<"End S  =" <<end_s << std::endl;
	highway_map.calculate_map_XYspline_for_s(end_s, 6, pts_x, pts_y,ref_yaw);
		
	
	
	double x_estimate = 30;
	double y_estimate = highway_map.get_y_from_curve(x_estimate);
	double dist_estimate = sqrt(x_estimate * x_estimate + y_estimate * y_estimate);
	
	double n_dist_inc = dist_estimate / (0.02*max_velocity);
	double dist_inc_x = x_estimate / n_dist_inc;
	double x_pt = 0;
	double y_pt;
	
	for(int i = 1; i<= 50 - path_size;i++){
		
		x_pt += dist_inc_x;
		y_pt = highway_map.get_y_from_curve(x_pt);

		
		next_x_vals.push_back(ref_x + (x_pt * cos(ref_yaw)  - y_pt * sin(ref_yaw)));
		next_y_vals.push_back(ref_y + (x_pt * sin(ref_yaw)  + y_pt * cos(ref_yaw)));

		
	}

}



