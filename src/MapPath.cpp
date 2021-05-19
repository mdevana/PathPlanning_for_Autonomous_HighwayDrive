#include <vector>
#include <math.h>
#include "MapPath.h"
#include <string>


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




