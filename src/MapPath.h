#ifndef MapPath_H
#define MapPath_H
#include <vector>
#include "helpers_planning.h"

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
  
  void Init();
  void set_map_path_data(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy);
  vector<waypoint> get_map_path_s(int ind);

 private:

  vector<waypoint> map_highway;
  vector<waypoint> path_discretised;

};

#endif  // MapPath_H