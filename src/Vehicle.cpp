//
// Created by John on 11/22/2018.
//

#include <iostream>
#include "Vehicle.h"
#include "helper_functions.h"
#include <math.h>
#include <algorithm>

#define LEFT -1
#define RIGHT 1
#define KEEP 0

using namespace std;


// Lane Speeds
	double lane_0_speed;
	double lane_1_speed;
	double lane_2_speed;

	double lane_0_clearance;
	double lane_1_clearance;
	double lane_2_clearance;

int fastest_lane()
{
	if ( (lane_0_speed > lane_1_speed) && (lane_0_speed > lane_2_speed))
		return 0;

	if ( (lane_1_speed > lane_0_speed) && (lane_1_speed > lane_2_speed))
		return 1;

	if ( (lane_2_speed > lane_0_speed) && (lane_2_speed > lane_1_speed))
		return 2;

	return 0;
}

//--------------------------
//      Find Lane Speeds
//--------------------------
	void Vehicle::find_lane_speeds(const vector<Vehicle> &other_cars)
	{
		// Lane Distances
			lane_0_clearance = std::numeric_limits<double>::max();
			lane_1_clearance = std::numeric_limits<double>::max();
			lane_2_clearance = std::numeric_limits<double>::max();

		// Lane Speeds
			for (auto other_car : other_cars)
			{
					switch (other_car.lane)
					{
						case 0:
							if ((other_car.s < lane_0_clearance) && (other_car.s >= this->s))
							{
								lane_0_clearance = (other_car.s - this->s);
								lane_0_speed = other_car.speed;
							}
							break;

						case 1:
							if ((other_car.s < lane_1_clearance) && (other_car.s >= this->s))
							{
								lane_1_clearance = (other_car.s - this->s);
								lane_1_speed = other_car.speed;
							}
							break;

						case 2:
							if ((other_car.s < lane_2_clearance) && (other_car.s >= this->s))
							{
								lane_2_clearance = (other_car.s - this->s);
								lane_2_speed = other_car.speed;
							}
							break;

						default:
							cout << "\n\n -- Error -- \n\n   other_car.lane = " << other_car.lane << endl;
							break;
					}
			}

	}




//--------------------------
//      Change Lane Cost
//--------------------------
	double Vehicle::change_lane_cost(const vector<Vehicle> &other_cars, int direction)
	{

		double cost = 0;
		double lane_speeds[] = {lane_0_speed, lane_1_speed, lane_2_speed};
		double lane_clearance[] = {lane_0_clearance, lane_1_clearance, lane_2_clearance};


		// Stay on the Road
			if ( (lane == 0) && (direction == LEFT))  { return 999.9;}
			if ( (lane == 2) && (direction == RIGHT)) { return 999.9;}

		// Is it secret? Is it safe?
			for (auto other_car : other_cars)
			{
				if ( (other_car.lane == lane + direction) && (other_car.s >= s - 0.5 ) && (other_car.s <= s + 0.5))
					return 999;
			}

		// Cost -- Lane speeds
			cost += (target_speed - lane_speeds[lane + direction]) / target_speed;
			cost -= (1 / lane_clearance[lane + direction]);


		return cost;

	}
//--------------------------
//      Calculate Cost
//--------------------------
	void Vehicle::calculate_cost(const vector<Vehicle> &other_cars)
	{

		// Costs
			double keep_lane_cost;
			double change_left_cost;
			double change_right_cost;

		// Flags
			bool safety_flag_left = true;
			bool safety_flag_right = true;

		// Lane Speeds
			this->find_lane_speeds(other_cars);
			double lane_speeds[] = { lane_0_speed, lane_1_speed, lane_2_speed};

		// Keep Lane Cost
			keep_lane_cost = change_lane_cost(other_cars, KEEP);

		// Change Lane Costs
			change_left_cost = change_lane_cost(other_cars, LEFT);
			change_right_cost = change_lane_cost(other_cars, RIGHT);


		// Evaluate Costs
			double costs[] = {change_left_cost, keep_lane_cost, change_right_cost};
			double *best_cost = min_element(costs, costs+3);


			cout << "\n" << endl;

			if (*best_cost == keep_lane_cost)
			{
				stay_in_lane();
				this->current_state = keep_lane;
				cout << "Stay in Lane " << lane << endl;
			}


			if (*best_cost == change_left_cost)
			{
				lane -= 1;
				this->target_speed = lane_speeds[lane - 1];
				this->current_state = change_left;
				cout << " Change Left " << lane << endl;
			}

			if (*best_cost == change_right_cost)
			{
				lane += 1;
				this->target_speed = lane_speeds[lane + 1];
				this->current_state = change_right;
				cout << "Change Right " << lane << endl;
			}

			cout << "Cost  = " << costs[0] << " -- " << costs[1] << " -- " << costs[2] << " :: " << *best_cost << endl;
			cout << "Speed = " << lane_speeds[0] << " -- " << lane_speeds[1] << " -- " << lane_speeds[2] << lane_speeds[lane] << endl;
			cout << "Clear = " << lane_0_clearance << " -- " << lane_1_clearance << " -- " << lane_2_clearance << endl;

	}


//--------------------------
//      Adjust Speed
//--------------------------
	void Vehicle::adjust_speed()
	{
		// Close Enough
			if (abs(speed - target_speed) < 0.1)
				return;

		// Adjust Speed
			speed < target_speed ? (speed += max_accl) : (speed -= max_accl);
	}

//--------------------------
//      Plan Next State
//--------------------------
	void Vehicle::plan_next_state( const vector<Vehicle> &other_cars)
	{
		// Finish Change before planning another change
			if (current_state != keep_lane)
				return;

		// Keep Lane
			if (front_distance > 2 * following_distance)
				stay_in_lane();

		// Calculate Costs
			else this->calculate_cost(other_cars);

	}

//--------------------------
//      Keep Lane
//--------------------------
	void Vehicle::stay_in_lane()
	{
		if (front_distance <= safety_buffer)
		{
			target_speed = leading_car_speed - 3;
			cout << "Safety Buffer" << endl;
		}

		else if ( (front_distance >= safety_buffer) && (front_distance <= following_distance) )
		{
			target_speed = leading_car_speed;
		}

		else if ( (front_distance >= following_distance) && (front_distance <= 2 * following_distance) )
		{
			target_speed = leading_car_speed + 2;
		}

		else {
			target_speed = 49.7;
		}

		adjust_speed();
	}
