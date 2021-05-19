#ifndef MapPath_H
#define MapPath_H
#include <vector>
#include "WayPoint.h"

using std::vector;

class MapPath {
 public:
  /**
   * Constructor
   */
  MapPath();

  /**
   * Destructor.
   */
  virtual ~MapPath();
  
  void Init_from_cloudpoints(MapPath map_points);
  void set_map_path_data(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy);
  vector<WayPoint> get_map_path_s(int s_val);
	
	
 private:

  vector<WayPoint> points_group;
  
  

};

#endif  // MapPath_H