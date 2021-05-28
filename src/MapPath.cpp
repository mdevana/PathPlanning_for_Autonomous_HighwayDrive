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

MapPath::MapPath() {}

MapPath::~MapPath() {}

void MapPath::Init_from_cloudpoints(MapPath map_points) {
  

  


}



void MapPath::set_map_path_data(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy ) {
  
    
  
  for (int i = 0; i < x.size(); ++i) {
	  
	  WayPoint w_p(x[i],y[i],s[i],dx[i],dy[i]);
	  points_group.push_back(w_p);
	  
  }
  
  x_spline.set_points(s,x,spline::cspline);
  y_spline.set_points(s,y,spline::cspline);
  dx_spline.set_points(s,dx,spline::cspline);
  dy_spline.set_points(s,dy,spline::cspline);
  
  
}

 
WayPoint MapPath::get_map_point_for_s(double s_val) {
	 
	 
     WayPoint wp_interpolated(x_spline(s_val), y_spline(s_val),s_val,dx_spline(s_val),dy_spline(s_val));
	 return(wp_interpolated);

}




WayPoint MapPath::get_map_convertedXY_for_s(double s_val, double d_val) {
	 
	 vector<double> x_vect;
	 vector<double> y_vect;
	 vector<double> s_vect;
	 

	 
	 for (WayPoint wp:points_group) {

	  x_vect.push_back(wp.get_x_co());
	  y_vect.push_back(wp.get_y_co());
	  s_vect.push_back(wp.get_s_co());
	  
	 }
	 
	 vector<double>  XY_1 = getXY(s_val, round(d_val), s_vect, x_vect, y_vect);
	 vector<double>  XY_2 = getXY(s_val+30, round(d_val), s_vect, x_vect, y_vect);
	 vector<double>  XY_3 = getXY(s_val+60, round(d_val), s_vect, x_vect, y_vect);
	 
	 std::cout <<"Current X vector calculated :" <<XY_1[0] <<" s = "<<s_val<< std::endl;
	 std::cout <<"Current X vector calculated :" <<XY_2[0] <<" s = "<<s_val<< std::endl;
	 std::cout <<"Current X vector calculated :" <<XY_3[0] <<" s = "<<s_val<< std::endl;
	 
	 std::cout <<"Current X vector calculated :" <<XY_1[0] <<" s = "<<s_val<< std::endl;
	 std::cout <<"Current X vector calculated :" <<XY_2[0] <<" s = "<<s_val<< std::endl;
	 std::cout <<"Current X vector calculated :" <<XY_3[0] <<" s = "<<s_val<< std::endl;
	 
	 vector<double> pts_x;
	 vector<double> pts_y;
	 vector<double> pts_s;
	 
	 pts_s.push_back(s_val);
	 pts_s.push_back(s_val+30);
	 pts_s.push_back(s_val+60);
	 
	 pts_x.push_back(XY_1[0]);
	 pts_x.push_back(XY_2[0]);
	 pts_x.push_back(XY_3[0]);
	 
	 pts_y.push_back(XY_1[1]);
	 pts_y.push_back(XY_2[1]);
	 pts_y.push_back(XY_3[1]);
	 
	  
	 spline xs_curve,ys_curve;
     
	 xs_curve.set_points(pts_x,pts_s,spline::cspline);
	 ys_curve.set_points(pts_y,pts_s,spline::cspline);
	 
     

	 double d_y= dy_spline(s_val);
	 if (abs(d_y) > 1)
		d_y = round(d_y);
	 double d_x= sin(acos(d_y));
	 
	 //std::cout <<"Current d vector calculated :" <<sqrt(d_x * d_x + d_y * d_y) << std::endl;
	 //std::cout <<"Current dx,dy vector :" <<d_x<<","<<d_y<<std::endl;
	 //std::cout <<"Lane Code :" <<d_val<<std::endl;
	 
     
	 //XY[0] = xs_curve(s_val);
	 //XY[1] = ys_curve(s_val);
	 
	 //d_val = 6.0;

	 WayPoint wp( xs_curve(s_val), ys_curve(s_val), s_val, d_x, d_y);    
	 return(wp);

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
	 vector<double>  XY{0,0};
	 vector<WayPoint>  pts_jerk_optimised;
	 double s_val,d_val;
	 double d_x, d_y;
	 
	 while(running_time < (start_time + end_time) ){
		 
		 
		
		s_val=Poly_eval_JMT(coeff_s,running_time);
		d_val=Poly_eval_JMT(coeff_d,running_time);
		
		XY[0] = x_spline(s_val);
	    XY[1] = y_spline(s_val);
		
		// define dx, dy
		double d_y= dy_spline(s_val);
		if (abs(d_y) > 1)
			d_y = round(d_y);
		double d_x= sin(acos(d_y));

	    pts_jerk_optimised.push_back(WayPoint( XY[0]+ (d_val) * d_x, XY[1] + (d_val) * d_y, s_val, d_x * d_val, d_y * d_val)); 
		
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

