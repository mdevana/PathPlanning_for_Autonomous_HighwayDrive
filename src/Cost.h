#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

float calculate_cost(const Vehicle &vehicle, 
                     const map<int, Vehicle> &predictions, 
                     const vector<Vehicle> &trajectory);

float goal_lane_cost(const Vehicle &vehicle,  
                         const vector<Vehicle> &trajectory,  
                         const map<int, Vehicle> &predictions, 
                         map<string, float> &data);

float inefficiency_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, Vehicle> &predictions, 
                        map<string, float> &data);
float lane_change_safety_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, Vehicle> &predictions, 
                        map<string, float> &data);

float lane_speed(const Vehicle &vehicle,const map<int, Vehicle> &predictions, int lane);

map<string, float> get_helper_data(const Vehicle &vehicle, 
                                   const vector<Vehicle> &trajectory, 
                                   const map<int, Vehicle> &predictions);

#endif  // COST_H