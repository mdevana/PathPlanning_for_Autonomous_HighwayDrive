#include <vector>
#include <math.h>
#include "MapPath.h"
#include <string>
#include "helpers_planning.h"
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"


using std::vector;
using std::string;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using tk::spline;



MapPath::MapPath() {}

MapPath::~MapPath() {}

void MapPath::Init_from_cloudpoints(MapPath map_points) {
  

  


}



void MapPath::set_map_path_data(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy ) {
  
    
  
  for (int i = 0; i < x.size(); ++i) {
	  
	  WayPoint w_p(x[i],y[i],s[i],dx[i],dy[i]);
	  points_group.push_back(w_p);
	  
  }

  
}

 





void MapPath::calculate_map_XYspline_for_s(double s_val, double d_val,vector<double> &prev_pts_x, vector<double> &prev_pts_y, double ref_yaw){
	 
	 vector<double> x_vect;
	 vector<double> y_vect;
	 vector<double> s_vect;
	 

	 
	 for (WayPoint wp:points_group) {

	  x_vect.push_back(wp.get_x_co());
	  y_vect.push_back(wp.get_y_co());
	  s_vect.push_back(wp.get_s_co());
	  
	 }
	 
	 vector<double>  XY_1 = getXY(s_val+30, round(d_val), s_vect, x_vect, y_vect);
	 vector<double>  XY_2 = getXY(s_val+60, round(d_val), s_vect, x_vect, y_vect);
	 vector<double>  XY_3 = getXY(s_val+90, round(d_val), s_vect, x_vect, y_vect);
	 
	 /*std::cout<< " pts_prev_x 0 ["<<prev_pts_x[0]<<std::endl;
	 std::cout<< " pts_prev_x 1 ["<<prev_pts_x[1]<<std::endl;
	 std::cout<< " pts_prev_XY_1 0 ["<<XY_1[0]<<std::endl;
	 std::cout<< " pts_prev_XY_1 0 ["<<XY_2[0]<<std::endl;
	 std::cout<< " pts_prev_XY_1 0 ["<<XY_3[0]<<std::endl;*/
	 
	 vector<double> pts_x;
	 vector<double> pts_y;
	
	 pts_x.push_back(prev_pts_x[0]);
	 pts_x.push_back(prev_pts_x[1]);
	 pts_x.push_back(XY_1[0]);
	 pts_x.push_back(XY_2[0]);
	 pts_x.push_back(XY_3[0]);
	 
	 
	 pts_y.push_back(prev_pts_y[0]);
	 pts_y.push_back(prev_pts_y[1]);
	 pts_y.push_back(XY_1[1]);
	 pts_y.push_back(XY_2[1]);
	 pts_y.push_back(XY_3[1]);
	 
	 /*for(int j=0; j< pts_x.size(); j++){
		 std::cout<< " pts_x ["<<j<<"] = "<<pts_x[j]<<std::endl;
		 std::cout<< " pts_y ["<<j<<"] = "<<pts_y[j]<<std::endl;
		 
	 }*/
	 
	 double ref_x = prev_pts_x[1];
	 double ref_y = prev_pts_y[1];
	 
	 double shift_x;
	 double shift_y;
	 
	 for(int i = 0;i< pts_x.size();i++){
		 
		 shift_x = pts_x[i]-ref_x;
		 shift_y = pts_y[i]-ref_y;
		 
		 /*std::cout<< " shift_x ["<<i<<"] = "<<shift_x<<std::endl;
		 std::cout<< " shift_y ["<<i<<"] = "<<shift_y<<std::endl;*/
		 
		 pts_x[i] = ( shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw) );
		 pts_y[i] = ( shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw) );
		 
		 
	 }
	 
	 for(int j=0; j< pts_x.size(); j++){
		 /*std::cout<< "transformed pts_x ["<<j<<"] = "<<pts_x[j]<<std::endl;
		 std::cout<< "transformed pts_x ["<<j<<"] = "<<pts_y[j]<<std::endl;*/
		 //std::cout<< " pts_y ["<<j<<"] = "<<pts_x[0];
		 
	 }
	 
	 xy_curve.set_points(pts_x,pts_y);
	 
	 


}

double MapPath::get_y_from_curve(double x){
	
	return xy_curve(x);
}



WayPoint MapPath::get_map_convertedS_for_XY(double x_val, double y_val, double theta) {
	 
	 vector<double> x_vect;
	 vector<double> y_vect;

	 for (WayPoint wp:points_group) {

	  x_vect.push_back(wp.get_x_co());
	  y_vect.push_back(wp.get_y_co());

	 }

     vector<double>  SD = getFrenet(x_val,y_val,theta,x_vect,y_vect);

	 WayPoint wp( x_val, y_val, SD[0], SD[1]);
	 return(wp);

}

vector<WayPoint> MapPath::get_map_convertedSD_for_XY_jerk_optimised(vector<double> &s_start,vector<double> &s_end, vector<double> &d_start, vector<double> &d_end, double start_time, double end_time, double inc) {
	 
	 vector<double> x_vect;
	 vector<double> y_vect;
	 vector<double> s_vect;

	 for (WayPoint wp:points_group) {

	  x_vect.push_back(wp.get_x_co());
	  y_vect.push_back(wp.get_y_co());
	  s_vect.push_back(wp.get_s_co());

	 }

	 vector<double> coeff_s=JMT(s_start, s_end, end_time - start_time);
	 vector<double> coeff_d=JMT(d_start, d_end, end_time - start_time);
	 
	 double running_time = start_time;
	
	 vector<WayPoint>  pts_jerk_optimised;
	 double s_val,d_val;
	 double d_x, d_y;
	 
	 while(running_time < (start_time + end_time) ){
		 
		 
		
		s_val=Poly_eval_JMT(coeff_s,running_time);
		d_val=Poly_eval_JMT(coeff_d,running_time);
		
		vector<double>  XY = getXY(s_val, d_val, s_vect, x_vect, y_vect);

	    pts_jerk_optimised.push_back(WayPoint( XY[0], XY[1], s_val, d_val)); 
		
		running_time += inc;
		 
	 }

	 return(pts_jerk_optimised);

}

vector<double> MapPath::JMT(vector<double> &start, vector<double> &end, double T) {
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

double MapPath::Poly_eval_JMT(vector<double> coeff, double t){
	
	return (coeff[0] + coeff[1] * t + coeff[2] * pow(t,2) + coeff[3] * pow(t,3) + coeff[4] * pow(t,4) + coeff[5] * pow(t,5));
	
}

