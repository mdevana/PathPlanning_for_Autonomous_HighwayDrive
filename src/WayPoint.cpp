#include <vector>
#include <math.h>
#include "WayPoint.h"
#include <string>


WayPoint::WayPoint() {}

WayPoint::WayPoint(double x, double y, double z,double s,double dx, double dy){
  
    x_co = x;
    y_co = y;
    s_co = s;
    dx_co = dx;
    dy_co = dy;
}


WayPoint::~WayPoint() {}





