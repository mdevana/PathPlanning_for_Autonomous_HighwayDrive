#ifndef PathGenerator_H
#define PathGenerator_H
#include <vector>
#include "MapPath.h"

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
  
  void Init(double inc, MapPath mp);

  vector<double> get_x_vals();
  vector<double> get_y_vals();
  
  void set_localization_data(double x,double y, double yaw);
  void set_previous_path_data(vector<double> x,vector<double> y);

  
  void generate_simple_path();
  void generate_circular_path();
  void generate_map_path();
  
  double car_x;
  double car_y;
  double car_yaw;
  
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  
  MapPath Map_Highway;
  
  

 private:

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  double dist_inc;

};

#endif  // PathGenerator_H