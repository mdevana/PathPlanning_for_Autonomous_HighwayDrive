#ifndef PathGenerator_H
#define PathGenerator_H
#include <vector>
#include "WayPoint.h"

using std::vector;

class PathGenerator {
 public:
  /**
   * Constructor
   */
  PathGenerator();

  /**
   * Destructor.
   */
  virtual ~PathGenerator();
  
  void Init(double,MapPath);

  vector<double> get_x_vals();
  vector<double> get_y_vals();
  
  void set_localization_data(double x,double y, double s, double d,double yaw,double speed);
  void set_previous_path_data(vector<double> x,vector<double> y);

  
  void generate_simple_path();
  void generate_circular_path();
  void generate_map_path();
  
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  
  
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_s;
  double end_d;
  
  
  //MapPath Map_Highway;

 private:

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  double dist_inc;
  double current_path_length;
  
  MapPath highway_map;
  MapPath highway_map_as_spline;
  
  vector<double> JMT(vector<double> &start, vector<double> &end, double T);

};

#endif  // PathGenerator_H