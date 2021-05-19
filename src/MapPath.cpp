#include <vector>
#include <math.h>
#include "MapPath.h"
#include <string>
#include "spline.h"

using std::vector;
using std::string;

MapPath::MapPath() {}

MapPath::~MapPath() {}

void MapPath::Init_from_cloudpoints(MapPath map_points) {
  
  vector<double> s_vector; 
  vector<double> x_vector; 
  vector<double> y_vector; 
  vector<double> dx_vector; 
  vector<double> dy_vector; 
  
  for(int i=0;i<map_points.points_group.size();i++){
	  s_vector[i] = map_points.points_group[i].get_s_co();
	  x_vector[i] = map_points.points_group[i].get_x_co();
	  y_vector[i] = map_points.points_group[i].get_y_co();
	  dx_vector[i] = map_points.points_group[i].get_dx_co();
	  dy_vector[i] = map_points.points_group[i].get_dy_co();
	  
  }
  
  //tk::spline s(map_points.points_group,Y,tk::spline::cspline); 

}



void MapPath::set_map_path_data(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy ) {
  
  int path_size = x.size();
  
  
  for (int i = 0; i < path_size; ++i) {
	  
	  WayPoint w_p(x[i],y[i],s[i],dx[i],dy[i]);
	  points_group.push_back(w_p);
	  
  }
}

 
vector<WayPoint> MapPath::get_map_path_s(int ind) {
	 
	 
     return (points_group);

}




