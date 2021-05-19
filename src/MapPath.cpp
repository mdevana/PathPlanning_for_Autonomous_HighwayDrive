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
  
  int path_size = x.size();
  
  
  for (int i = 0; i < path_size; ++i) {
	  
	  WayPoint w_p(x[i],y[i],s[i],dx[i],dy[i]);
	  points_group.push_back(w_p);
	  
  }
  
  x_spline.set_points(s_vector,x_vector,spline::cspline);
  y_spline.set_points(s_vector,y_vector,spline::cspline);
  dx_spline.set_points(s_vector,dx_vector,spline::cspline);
  dy_spline.set_points(s_vector,dy_vector,spline::cspline);
  
}

 
vector<WayPoint> MapPath::get_map_path_s(int ind) {
	 
	 
     return (points_group);

}




