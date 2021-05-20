#include <vector>
#include <math.h>
#include "WayPoint.h"
#include <string>


WayPoint::WayPoint() {}

WayPoint::WayPoint(double x, double y,double s,double dx, double dy){
  
    x_co = x;
    y_co = y;
    s_co = s;
    dx_co = dx;
    dy_co = dy;
}

WayPoint::WayPoint(double x, double y,double s,double d){
  
    x_co = x;
    y_co = y;
    s_co = s;
    d_co = d;
    
}

double WayPoint::get_x_co() { return x_co;}
double WayPoint::get_y_co() { return y_co;}
double WayPoint::get_s_co() { return s_co;}
double WayPoint::get_dx_co() { return dx_co;}
double WayPoint::get_dy_co() { return dy_co;}
double WayPoint::get_d_co() { return sqrt(dy_co * dy_co + dx_co * dx_co );}
double WayPoint::get_d_val() { return d_co;}


WayPoint::~WayPoint() {}





