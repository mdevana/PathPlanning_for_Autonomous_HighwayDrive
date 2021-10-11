#include "Vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "Cost.h"
#include <iostream>
#include <math.h>

using std::string;
using std::vector;
using std::cout;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int id, double x, double y,double s, double d, double vx, double vy, string state) {
  // Constructor for general initialisation of a vehicle
  this->ID = id;
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->vx = vx;
  this->vy = vy;
  this->state = state;
  this->v = sqrt(vx*vx+vy*vy);
  this->lane = getlanefrom_d(d);

}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {
  // constructor to initisalise predicted vehicles and trajectories
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state=state;

}

Vehicle::Vehicle(double car_x, double car_y, double car_s, double car_d, double car_speed, double car_accl, double car_yaw, string state, double max_velocity ){
  // constructor to initialise ego vehicle	
  this->ID = 1000;
  
  this->lane = getlanefrom_d(car_d);
  this->d = car_d;
  this->s = car_s;
  
  this->v = car_speed;
  
  this->yaw = (car_yaw) * M_PI / 180;
  this->x = car_x;
  this->y = car_y;
    
  this->state = state;
  this->target_speed = max_velocity;
  this->a = car_accl;
	
}


Vehicle::~Vehicle() {}


int Vehicle::getlanefrom_d(double d){
	// caluclate lane number from d coordinate value
	if (d>0 && d<=4)
		return (1);
	else if (d > 4 && d<=8)
		return (2);
	else if ( d > 8 && d<=12)
		return (3);
	
}

void Vehicle::VehicleParamDisplay(){
/** function to print out stored values in the object. useful for printing data read thtrough sensor fusion **/	
	std::cout <<"Vehicle ID =" <<this->ID << std::endl;
	std::cout <<"X_Pos =" <<this->x << std::endl;
	std::cout <<"Y_Pos =" <<this->y << std::endl;
	std::cout <<"s =" <<this->s << std::endl;
	std::cout <<"d =" <<this->d << std::endl;
	std::cout <<"lane =" <<this->lane << std::endl;
	std::cout <<"vx =" <<this->vx << std::endl;
	std::cout <<"vy =" <<this->vy << std::endl;
	std::cout <<"v =" <<this->v << std::endl;
}

vector<string> Vehicle::successor_states(map<int, Vehicle> &predictions, double time_span) {
  // Provides the all availble state given the current state for the FSM 
  // Cost will be calculated for all states and state with lowest cost will be used for next step.
 
  vector<string> states;
  states.push_back("KL");
  states.push_back("PLCL");
  states.push_back("PLCR");
  states.push_back("LCR");
  states.push_back("LCL");

  return states;
}

bool Vehicle::get_vehicle_ahead(map<int, Vehicle> &predictions, 
                                int in_lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found ahead of the current vehicle within 50 m in given lane, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  double min_s = 100000;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  
  for (map<int, Vehicle>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    temp_vehicle = it->second;
	  
	  if (temp_vehicle.lane == in_lane)
	  if ( (temp_vehicle.s > this->s ) && (temp_vehicle.s - this->s < 50) && (temp_vehicle.s < min_s) ) {
		min_s = temp_vehicle.s;
		rVehicle = temp_vehicle;
		found_vehicle = true;
	}
  }
  
  
  
  return found_vehicle;
}

bool Vehicle::get_vehicle_behind(map<int, Vehicle> &predictions, 
                                 int in_lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found behind the current vehicle within 50 m, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, Vehicle>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    temp_vehicle = it->second;
    if (temp_vehicle.lane == in_lane)
      if (temp_vehicle.s < this->s && (this->s - temp_vehicle.s < 50) && (temp_vehicle.s > max_s) ) {
		max_s = temp_vehicle.s;
		rVehicle = temp_vehicle;
		found_vehicle = true;
    }
  }
  
  return found_vehicle;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, 
                                                map<int, Vehicle> &predictions, double time_span) {
  
  // Generate a lane change trajectory.
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  if (new_lane < 1 || new_lane >3)
	return trajectory;
  
  
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies 
  //   that corridor in a given lane.
  for (map<int, Vehicle>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    next_lane_vehicle = it->second;
	double forward_clearance = this->s + 20;
	double backward_clearance = this->s - 10;
	
    if ( (next_lane_vehicle.s  < forward_clearance) && (next_lane_vehicle.s > backward_clearance) && next_lane_vehicle.lane == new_lane) {
      // If lane change is not possible, return empty trajectory.
	  
      return trajectory;
    }
  }
  trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, 
                               this->state));
  vector<float> kinematics = get_kinematics(predictions, new_lane,time_span);
  trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], 
                               kinematics[2], state));
  return trajectory;
}


vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, 
                                                     map<int, Vehicle> &predictions, double time_span) {
  
  // Generate a trajectory preparing for a lane change.
  float new_s;
  float new_v;
  float new_a;
  Vehicle vehicle_behind;
  vector<Vehicle> trajectory;
  int new_lane = this->lane + lane_direction[state];
  if (new_lane < 1 || new_lane >3)
	return trajectory;
  
  trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, 
                                        this->state));
  vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane, time_span);

  if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];    
  } else {
    vector<float> best_kinematics;
    vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane, time_span);
    // Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
  
  return trajectory;
}

vector<Vehicle> Vehicle::constant_speed_trajectory(double time_span) {
  // Generate a constant speed trajectory.
  float next_pos = this->s + this->v * time_span + 0.5 * this->a * time_span * time_span;
  vector<Vehicle> trajectory = {Vehicle(this->lane,this->s,this->v,this->a,this->state), 
                                Vehicle(this->lane,next_pos,this->v,0,this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, Vehicle> &predictions, double time_span) {
  
  // Generate a keep lane trajectory.
  vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
  vector<float> kinematics = get_kinematics(predictions, this->lane, time_span);
  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];
  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));


  
  return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, Vehicle> &predictions, 
                                      int lane, double time_span) {
  // Gets next timestep kinematics (position, velocity, acceleration) 
  //   for a given lane. Tries to choose the maximum velocity and acceleration, 
  //   given other vehicle positions and accel/velocity constraints.051080k
  float max_velocity_accel_limit = this->max_acceleration * time_span + this->v;
  float min_velocity_accel_limit = this->v - this->max_acceleration * time_span ;
  float max_velocity_in_front;
  float new_position;
  float new_velocity;
  float new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
	
	double allowed_gap_to_front_vehicle = (vehicle_ahead.s - this->s - this->preferred_buffer); 
    //std::cout<< " Gap maintained with vehicle id  "<<vehicle_ahead.ID<<std::endl; 	
    
	if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
      // Ego vehcile sandwiched between front and back vehicle. Must travel at the speed of traffic, regardless of preferred/minimum buffer distance
	  	  
	  if (allowed_gap_to_front_vehicle > 0){
		// increase Speed to that of front vehicle provided there is gap in front
		if (this->v < vehicle_ahead.v) {
			new_velocity = max_velocity_accel_limit; 		
			if (max_velocity_accel_limit > vehicle_ahead.v)
				new_velocity = vehicle_ahead.v;
		}
		else
			new_velocity = this->v;


	  }
		
	else {
		//either gap is critical or vehicle is going slow than 
			
			//new_velocity = this->v - 4 * time_span;
			if (this->v > vehicle_ahead.v){ 
				new_velocity = min_velocity_accel_limit;
				if (min_velocity_accel_limit < vehicle_ahead.v)
					new_velocity = getVelocityChecked(vehicle_ahead.v,this->v,time_span,min_velocity_accel_limit);
			}
			else 
				new_velocity = this->v;


	  }
		
    } else {
	  // Ego has vehicle only in front and no vehilce in rear. Adjust speed to follow the front vehicle by either closing gap or follow vehicle ahead speed
	  //std::cout<< " allowed_gap_to_front_vehicle "<<allowed_gap_to_front_vehicle<<std::endl;
      	  
      if (allowed_gap_to_front_vehicle > 0 ){	  
		 
		
		// To clear traffic , vehicle can close the gap to minimum buffer and look for opportunity to overtake : CODE is not Activated 
				
		double braking_dist = abs((vehicle_ahead.v * vehicle_ahead.v) - (this->v * this->v)) / (2 * this->max_acceleration) ; // Equation v*v -u*u / 2 * a * s
		
		if (braking_dist >= allowed_gap_to_front_vehicle){
		
		    
			new_velocity = min_velocity_accel_limit;
			if (min_velocity_accel_limit < vehicle_ahead.v)
				new_velocity = getVelocityChecked(vehicle_ahead.v,this->v,time_span,min_velocity_accel_limit);
			//std::cout<< " speed set in module "<<new_velocity<<std::endl; 
			
		}			
		else {
		    //std::cout<< " Recovering from ful brakes "<<allowed_gap_to_front_vehicle<<std::endl;
			new_velocity = max_velocity_accel_limit;	
			if (max_velocity_accel_limit > this->target_speed)
				new_velocity = this->target_speed;
		}
		
	  }
	  else {
		  // if allowed gap tp front vehicle is critical then reduce speed or follow front vehicle
		  //std::cout<< " allowed_gap_to_front_vehicle negative "<<allowed_gap_to_front_vehicle<<std::endl;
		  if (this->v > vehicle_ahead.v){
			  new_velocity = min_velocity_accel_limit;
			if (min_velocity_accel_limit < vehicle_ahead.v)
				new_velocity = getVelocityChecked(vehicle_ahead.v,this->v,time_span,min_velocity_accel_limit);
		  }
		  else
			  new_velocity = this->v;
		  
			
	  }
	  
    }
  } else {
	// No front and rear vehicles. Increase to target speed with allowed acceleration   
    new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
	
  }
    
  new_accel = (new_velocity - this->v) / time_span; // Equation: (v_1 - v_0)/t = acceleration
  new_position = this->s + new_velocity * time_span + new_accel * time_span * time_span / 2.0; // Equation: s = v *t + 0.5 * a * t * t
   
  return{new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::generate_trajectory(string state, 
                                             map<int, Vehicle> &predictions, double time_span) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory(time_span);
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions,time_span);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions, time_span);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions, time_span);
  }

  return trajectory;
}

vector<Vehicle> Vehicle::test_func(map<int, Vehicle> &predictions, double time_span){

  /**
   * Here the code for  transition_function  is implemented from the Behavior 
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and predictions. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.

   */
	
	vector<Vehicle> trajectory_for_state;
	vector<float> cost_for_trajectory;
    vector<vector<Vehicle>> final_trajectories;
	
	// Loop through all trajectories, evaluate them and find the trajectory with lowest cost
	vector<string> p_s_states =successor_states(predictions,time_span);
	float cost;
	for (vector<string>::iterator t=p_s_states.begin(); t!=p_s_states.end(); ++t) {
       
	   trajectory_for_state=generate_trajectory(*t,predictions,time_span);
	   	   
       if (trajectory_for_state.size() > 0) {
		cost = calculate_cost(*this,predictions,trajectory_for_state);
		cost_for_trajectory.push_back(cost);
		final_trajectories.push_back(trajectory_for_state);		
	   }

   }
   
    vector<float>::iterator min_cost=std::min_element(cost_for_trajectory.begin(),cost_for_trajectory.end());
    int best_index = std::distance(cost_for_trajectory.begin(),min_cost);
	
	return (final_trajectories[best_index]);
		
}


void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
  // Sets state and kinematics for ego vehicle using the last state of the trajectory.
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
  this->a = next_state.a;
  
  std::cout<< " new Velocity : "<<this->v<<std::endl;
  if (std::abs(this->a) > 9)
	std::cout<< " new accl : "<<this->a<<std::endl;
  
  
}


void Vehicle::generate_predictions(double time_span) {
  // Generates predictions for non-ego vehicles in the given time span to be used in trajectory predictions
  // Uses a linear point model
  this->s = this->s + (time_span * this->v);
}

void Vehicle::configure(double max_speed,int lane_avail, double max_accl) {
  // Sets various parameters which will impact the ego vehicle.
  target_speed = max_speed;
  lanes_available = lane_avail;  
  max_acceleration = max_accl;
}

double getVelocityChecked(double vel_to_reach,double current_velocity,double time_span,double next_step_velocity){
	double safe_accl_value = 9;
	accl = (vel_to_reach - current_velocity) / time_span;
	if (std::abs(accl) > safe_accl_value )
		return next_step_velocity;
	else
		return vel_to_reach;
		
	
}


