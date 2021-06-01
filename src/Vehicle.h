#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int id, double x, double y,double s, double d, double vx, double vy,string state); // constructor for initialise Vehicles from Sensor Fusion 
  Vehicle(int lane, float s, float v, float a, string state="CS"); // 
  Vehicle(int id,int lane, double car_x, double car_y, double car_s, double car_d, double car_speed, double car_yaw, string state="CS");// constructor for initialise Vehicles Ego Vehicle 

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  
  vector<double> cold_start(int n,double time_step);
  
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);

  

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> &predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> &predictions);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane, 
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane, 
                         Vehicle &rVehicle);

  void generate_predictions(int time_horizon, double simulator_time_step);

  void realize_next_state(vector<Vehicle> &trajectory);

  //void configure(vector<int> &road_data);

  // public Vehicle variables
  /*struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };*/
  //object functions
  vector<string> successor_states();
  
  void VehicleParamDisplay();
  
  
  
  
  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, 
                                     {"LCR", -1}, {"PLCR", -1}};

  int ID;
  double x, y;
  double yaw;
  double vx,vy,v;
  double s,d;
  double a;
  
  string state;
  
  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane, goal_lane, goal_s, lanes_available;

  float target_speed, max_acceleration = 9;

  
};

#endif  // VEHICLE_H