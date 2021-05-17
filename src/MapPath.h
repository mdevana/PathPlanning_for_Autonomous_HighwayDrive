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
  
  //void Init();
  //void set_map_path_data(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy);
  //vector<WayPoint> get_map_path_s(int ind);

 private:

  //vector<WayPoint> map_of_highway;
  

};

#endif  // MapPath_H