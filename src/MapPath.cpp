#include "MapPath.h"
#include <vector>
#include <math.h>
#include "helpers_planning.h"
#include <string>

using std::vector;
using std::string;

MapPath::MapPath() {}

MapPath::~MapPath() {}

void MapPath::Init() {
  
  

}



void MapPath::set_map_path_data(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy ) {
  
  int path_size = x.size();
  waypoint w_p;
  
  for (int i = 0; i < path_size; ++i) {
	  
	  w_p.x_co = x[i];
	  w_p.y_co = y[i];
      w_p.s_co = s[i];
      w_p.dx_co = dx[i];
      w_p.dy_co = dy[i];
	  map_highway.push_back(w_p);
	  
  }
}

 
vector<waypoint> MapPath::get_map_path_s(int ind) {
	 
	 path_discretised.clear();
	 
	 for(i = 0 ; i < ind ; i++) {
		 path_discretised.push_back(map_highway[i]);
		 
	 }
	 
  

}




