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
  
  double get_x_co();
  double get_y_co();
  double get_s_co();
  double get_dx_co();
  double get_dy_co();
  double get_d_co(); 

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