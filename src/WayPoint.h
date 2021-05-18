#ifndef WayPoint_H
#define WayPoint_H
#include <vector>


using std::vector;

class WayPoint {
 public:
  /**
   * Constructor
   */
  WayPoint();
  
  WayPoint(double x, double y,double s,double dx, double dy);

  /**
   * Destructor.
   */
  virtual ~WayPoint();
  

 private:


  double  x_co;
  double  y_co;
  double  s_co;
  double  dx_co;
  double  dy_co;

};

#endif  // WayPoint_H