#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
#include <iostream>

using std::string;
using std::vector;
using std::cout;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int id, double x, double y,double s, double d, double vx, double vy, string state) {
  this->ID = id;
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->vx = vx;
  this->vy = vy;
  max_acceleration = -1;
}

Vehicle::~Vehicle() {}

Vehicle::VehicleParamDisplay(){
	std::cout <<"Vehicle ID =" <<this->ID << std::endl;
	std::cout <<"X_Pos =" <<this->x << std::endl;
	std::cout <<"Y_Pos =" <<this->y << std::endl;
	std::cout <<"s =" <<this->s << std::endl;
	std::cout <<"d =" <<this->d << std::endl;
	std::cout <<"vx =" <<this->vx << std::endl;
	std::cout <<"vy =" <<this->vy << std::endl;
}
