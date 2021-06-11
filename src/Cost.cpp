#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>


using std::string;
using std::vector;


const float GOAL_LANE = 0.15;
const float EFFICIENCY = 0.75;
const float SAFE_LANE_CHANGE = 0.1;
const float SAFE_DIST_CHANGE = 0.0;



//   The weighted cost over all cost functions is computed
//   in calculate_cost. The data from get_helper_data will be used in 
//   the implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.

float goal_lane_cost(const Vehicle &vehicle, 
                         const vector<Vehicle> &trajectory, 
                         const map<int, Vehicle> &predictions, 
                         map<string, float> &data) {
  // Cost increases based on distance of intended lane (for planning a lane 
  //   change) and final lane of trajectory.
  // Cost of being out of goal lane also becomes larger as vehicle approaches 
  //   goal distance.
  // This function is very similar to what you have already implemented in the 
  //   "Implement a Cost Function in C++" quiz.
  float cost;
  float distance = data["distance_to_goal"];
  if (distance > 0) {
    cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] 
         - data["final_lane"])));
  } else {
    cost = 1;
  }

  return cost;
}

float inefficiency_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, Vehicle> &predictions, 
                        map<string, float> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane. 
  // This function is very similar to what you have already implemented in 
  //   the "Implement a Second Cost Function in C++" quiz.
  float proposed_speed_intended = lane_speed(vehicle,predictions, data["intended_lane"]);
  float proposed_speed_final = lane_speed(vehicle,predictions, data["final_lane"]);
 
    
  float cost = (2.0*vehicle.target_speed - proposed_speed_intended 
             - proposed_speed_final)/vehicle.target_speed;
  
  /*std::cout <<"Vehicle target Speed : " <<vehicle.target_speed<<std::endl;
  std::cout <<"proposed_speed_intended : " <<proposed_speed_intended<<std::endl;
  std::cout <<"proposed_speed_final : " <<proposed_speed_final<<std::endl;*/
  
  return cost;
}

float lane_change_safety_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, Vehicle> &predictions, 
                        map<string, float> &data) {

// cost is high if speed difference between current lane and final lane is less than 7 
    float speed_diff = abs(data["speed_final_lane"] - data["speed_current_lane"]);
	if ( speed_diff < 7 && data["speed_final_lane"]!= vehicle.target_speed && data["speed_current_lane"]!= vehicle.target_speed )
		return (1 * (7-speed_diff)/7);
	else
		return (0);


}

float lane_change_safe_dist_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, Vehicle> &predictions, 
                        map<string, float> &data) {

// cost is high if difference in distance with front vehicle between current lane and final lane is less than 50
// This function is thought to prevent waggling of ego vehicle between lanes when front vehicle in adjacent lanes travel with same s and velocity
    float distance_diff = abs(data["distance_to_front_car_final_lane"] - data["distance_to_front_car_current_lane"]);
	if (distance_diff<=50 && data["distance_to_front_car_final_lane"]<=50 && data["distance_to_front_car_current_lane"]<=50)
		return (1 - distance_diff/50.0);
	else 
        return 0; 


}


float lane_speed(const Vehicle &vehicle,const map<int, Vehicle> &predictions, int lane) {
  // All non ego vehicles in a lane have the same speed, so to get the speed 
  //   limit for a lane, we can just find one vehicle in that lane.
  
  Vehicle v_ahead;
  Vehicle veh = vehicle;
  map<int, Vehicle> predictions2 = predictions;
  
	bool v_ah = veh.get_vehicle_ahead(predictions2,lane,v_ahead);
	if (v_ah == true){
		
		return v_ahead.v;	
		
	}
	return (vehicle.target_speed);
	
  
  
}

float calculate_cost(const Vehicle &vehicle, 
                     const map<int, Vehicle> &predictions, 
                     const vector<Vehicle> &trajectory) {
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, 
                                                       predictions);
  float cost = 0.0;

  // Add additional cost functions here.
  vector<std::function<float(const Vehicle &, const vector<Vehicle> &, 
                             const map<int, Vehicle> &, 
                             map<string, float> &)
    >> cf_list = {inefficiency_cost,goal_lane_cost,lane_change_safety_cost,lane_change_safe_dist_cost};
  vector<float> weight_list = {EFFICIENCY,GOAL_LANE,SAFE_LANE_CHANGE,SAFE_DIST_CHANGE};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, 
                                               trajectory_data);
	if (i==3)
		std::cout <<"calculated lane distance cost : " <<cf_list[i](vehicle, trajectory, predictions, trajectory_data)<<std::endl;
	if (i==2)
		std::cout <<"calculated lane speed cost : " <<cf_list[i](vehicle, trajectory, predictions, trajectory_data)<<std::endl;
    cost += new_cost;
  }

  return cost;
}

map<string, float> get_helper_data(const Vehicle &vehicle, 
                                   const vector<Vehicle> &trajectory, 
                                   const map<int, Vehicle> &predictions) {
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or 
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory.
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help 
  //   differentiate between planning and executing a lane change in the 
  //   cost functions.
  map<string, float> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  float intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane - 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } else {
    intended_lane = trajectory_last.lane;
  }

  float distance_to_goal = vehicle.goal_s - trajectory_last.s;
  float final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
  trajectory_data["distance_to_front_car_current_lane"] = 1000;
  trajectory_data["distance_to_front_car_final_lane"] = 1000;
  
  Vehicle v_ahead;
  Vehicle veh = vehicle;
  map<int, Vehicle> predictions2 = predictions;
  
  trajectory_data["Speed_final_lane"] = vehicle.target_speed;
  bool v_ah = veh.get_vehicle_ahead(predictions2,final_lane,v_ahead);
	if (v_ah == true){
		//std::cout <<"lane _speed " <<v_ahead.v <<lane<<std::endl;
		 trajectory_data["Speed_final_lane"]=v_ahead.v;	
		 trajectory_data["distance_to_front_car_final_lane"] = abs(vehicle.s - v_ahead.s) ;
	}
    
  trajectory_data["Speed_current_lane"] = vehicle.target_speed;
  v_ah = veh.get_vehicle_ahead(predictions2,trajectory[0].lane,v_ahead);
	if (v_ah == true){
		//std::cout <<"lane _speed " <<v_ahead.v <<lane<<std::endl;
		 trajectory_data["Speed_current_lane"]=v_ahead.v;
		 trajectory_data["distance_to_front_car_current_lane"] = abs(vehicle.s - v_ahead.s) ;
		
	}

    
  return trajectory_data;
}