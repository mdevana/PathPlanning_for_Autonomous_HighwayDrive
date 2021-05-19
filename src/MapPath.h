#ifndef MapPath_H
#define MapPath_H
#include <vector>
#include "WayPoint.h"
#include "spline.h"

using std::vector;
using tk::spline;

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
  WayPoint get_map_point_for_s(int s_val);
  WayPoint get_map_convertedXY_for_s(int s_val);
	
	
 private:

  vector<WayPoint> points_group;
  
  spline x_spline;
  spline y_spline;
  spline dx_spline;
  spline dy_spline;
  
  

};

#endif  // MapPath_H