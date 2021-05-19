#include <vector>
#include <math.h>
#include "MapPath.h"
#include <string>
#include "helpers_planning.h"


using std::vector;
using std::string;

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

 
WayPoint MapPath::get_map_point_for_s(int s_val) {
	 
	 
     WayPoint wp_interpolated(x_spline(s_val), y_spline(s_val),s_val,dx_spline(s_val),dy_spline(s_val));
	 return(wp_interpolated);

}

WayPoint MapPath::get_map_convertedXY_for_s(int s_val) {
	 
	 vector<double> x_vect;
	 vector<double> y_vect;
	 vector<double> s_vect;
	 for (int i = 0; i < points_group.size(); ++i) {
	  
	  x_vect[i] = points_group[i].get_x_co();
	  y_vect[i] = points_group[i].get_y_co();
	  s_vect[i] = points_group[i].get_s_co();
	  
	 }
	 
	 double d_x= dx_spline(s_val);
	 double d_y= dy_spline(s_val);
	 
     vector<double>  XY = getXY(s_val, sqrt(d_x * d_x + d_y * d_y),s_vect, x_vect, y_vect);
	 
	 WayPoint wp( XY[0], XY[1], s_val, d_x,d_y);
	 return(wp);

}




