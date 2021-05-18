#include <vector>
#include <math.h>
#include "MapPath.h"
#include <string>

using std::vector;
using std::string;

MapPath::MapPath() {}

MapPath::~MapPath() {}

/*void MapPath::Init() {
  
  

}*/



void MapPath::set_map_path_data(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy ) {
  
  int path_size = x.size();
  
  
  for (int i = 0; i < path_size; ++i) {
	  
	  WayPoint w_p(x[i],y[i],s[i],dx[i],dy[i]);
	  map_of_highway.push_back(w_p);
	  
  }
}

 
vector<WayPoint> MapPath::get_map_path_s(int ind) {
	 
	 
     return (map_of_highway);

}




