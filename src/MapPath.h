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
  WayPoint get_map_convertedXY_for_s(double s_val, double d_val);
  WayPoint get_map_convertedS_for_XY(double x_val, double y_val, double theta);
  vector<WayPoint> get_map_convertedSD_for_XY_jerk_optimised(vector<double> &s_start,vector<double> &s_end, vector<double> &d_start, vector<double> &d_end, double start_time, double end_time, double inc);
  double Poly_eval_JMT(vector<double> coeff, double t);
  
  WayPoint get_map_point_for_x(double x_val);
  
  void calculate_map_XYspline_for_s(double s_val, int d_val,vector<double> &prev_pts_x, vector<double> &prev_pts_y, double ref_yaw, int lane);
  
  double get_y_from_curve(double x);
	
	
 private:

  vector<WayPoint> points_group;
  
  //spline x_spline;
  //spline y_spline;
  //spline dx_spline;
  //spline dy_spline;
  
  spline xy_curve;
  
  
  vector<double> JMT(vector<double> &start, vector<double> &end, double T);
  

};

#endif  // MapPath_H